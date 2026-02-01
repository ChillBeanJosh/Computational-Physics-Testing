using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;

public class GPUSPH3D : MonoBehaviour
{
    [Header("Capacity (GPU buffers)")]
    [SerializeField, Range(256, 500000)] int particleCapacity = 20000;

    [Header("Shared SPH Parameters")]
    [SerializeField] float smoothingRadius = 0.2f;
    [SerializeField] float particleRadius = 0.09f;

    [Header("Simulation")]
    [SerializeField] Vector3 gravityWorld = new Vector3(0, -9.81f, 0);
    [SerializeField] float timeStep = 0.006f;
    [SerializeField, Range(0f, 1f)] float boundaryDamping = 0.5f;

    [Header("Tank (required)")]
    [SerializeField] BoxCollider tankCollider;

    [Header("GPU Grid")]
    [SerializeField, Min(1)] int maxParticlesPerCell = 64;

    [Header("Spawner")]
    [SerializeField] Vector3 spawnCenterWorld = new Vector3(0, 2, 0);
    [SerializeField, Min(1)] int spawnWidth = 27;
    [SerializeField, Min(1)] int spawnHeight = 27;
    [SerializeField, Min(1)] int spawnDepth = 27;
    [SerializeField, Range(0.05f, 2f)] float spawnSpacingMultiplier = 0.6f;
    [SerializeField] bool spawnAsSphere = true;
    [SerializeField] bool appendSpawns = true;

    [System.Serializable]
    public struct FluidSpawnKey
    {
        public FluidLibrary.FluidType fluid;
        public KeyCode key;
    }

    [Header("Spawn Keys (Expandable)")]
    [SerializeField]
    FluidSpawnKey[] spawnKeys =
    {
        new FluidSpawnKey { fluid = FluidLibrary.FluidType.Water,  key = KeyCode.Alpha1 },
        new FluidSpawnKey { fluid = FluidLibrary.FluidType.Oil,    key = KeyCode.Alpha2 },
        new FluidSpawnKey { fluid = FluidLibrary.FluidType.Honey,  key = KeyCode.Alpha3 },
        new FluidSpawnKey { fluid = FluidLibrary.FluidType.Custom, key = KeyCode.Alpha4 },
    };

    [Header("Rendering")]
    [SerializeField] Material material;
    [SerializeField] Mesh mesh;
    [SerializeField] float renderStep = 0.05f;

    [Header("Compute")]
    [SerializeField] ComputeShader sphCompute;

    [Header("Optional scene BoxColliders (GPU OBB collisions)")]
    [SerializeField] bool enableSceneBoxColliders = false;
    [SerializeField] LayerMask sceneBoxLayer = ~0;
    [SerializeField, Min(0f)] float sceneBoxesRefreshInterval = 0.25f;

    [Header("Render culling bounds")]
    [SerializeField] float renderBoundsMargin = 2f;

    //Buffers:
    ComputeBuffer particleBuffer;
    ComputeBuffer positionsBuffer;
    ComputeBuffer fluidInfoBuffer;   
    ComputeBuffer cellCountsBuffer;
    ComputeBuffer cellIndicesBuffer;
    ComputeBuffer boxesBuffer;
    ComputeBuffer fluidsBuffer;      

    //Kernels:
    int kClearGrid, kBuildGrid, kDensityPressure, kForces, kIntegrate;

    //IDs:
    static readonly int
        particleCountId = Shader.PropertyToID("_ParticleCount"),
        timeStepId = Shader.PropertyToID("_TimeStep"),
        gravityLocalId = Shader.PropertyToID("_GravityLocal"),

        smoothingRadiusId = Shader.PropertyToID("_SmoothingRadius"),
        particleRadiusId = Shader.PropertyToID("_ParticleRadius"),

        boundsMinLocalId = Shader.PropertyToID("_BoundsMinLocal"),
        boundsMaxLocalId = Shader.PropertyToID("_BoundsMaxLocal"),
        boundaryDampingId = Shader.PropertyToID("_BoundaryDamping"),

        poly6ConstId = Shader.PropertyToID("_Poly6Constant"),
        spikyConstId = Shader.PropertyToID("_SpikyGradConstant"),
        viscConstId = Shader.PropertyToID("_ViscLapConstant"),

        localToWorldId = Shader.PropertyToID("_LocalToWorld"),

        gridSizeId = Shader.PropertyToID("_GridSize"),
        gridOriginLocalId = Shader.PropertyToID("_GridOriginLocal"),
        cellSizeId = Shader.PropertyToID("_CellSize"),
        maxPerCellId = Shader.PropertyToID("_MaxParticlesPerCell"),

        particlesId = Shader.PropertyToID("_Particles"),
        positionsId = Shader.PropertyToID("_Positions"),
        fluidInfoId = Shader.PropertyToID("_FluidInfo"),
        cellCountsId = Shader.PropertyToID("_CellCounts"),
        cellIndicesId = Shader.PropertyToID("_CellIndices"),

        boxesId = Shader.PropertyToID("_Boxes"),
        boxCountId = Shader.PropertyToID("_BoxCount"),

        fluidsId = Shader.PropertyToID("_Fluids"),
        fluidCountId = Shader.PropertyToID("_FluidCount"),

        stepId = Shader.PropertyToID("_Step");

    [StructLayout(LayoutKind.Sequential)]
    struct ParticleData
    {
        public Vector4 pos;
        public Vector4 vel;
        public Vector4 force;
        public float density;
        public float pressure;
        public uint typeId;
        public float mass;
        public Vector4 pad;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct FluidParamsGPU
    {
        public float restDensity;
        public float gasConstant;
        public float viscosity;
        public float pad;
    }

    int activeCount = 0;
    int cellCount = 0;
    int gridX, gridY, gridZ;

    Vector3 tankBoundsMinLocal;
    Vector3 tankBoundsMaxLocal;
    Vector3 tankGridOriginLocal;

    float boxesTimer = 0f;

    public int ActiveParticleCount => activeCount;
    public int ParticleCapacity => particleCapacity;

    void OnEnable()
    {
        if (tankCollider == null || sphCompute == null || material == null || mesh == null)
        {
            Debug.LogError($"{nameof(GPUSPH3D)}: Missing references (tankCollider/compute/material/mesh).");
            enabled = false;
            return;
        }

        kClearGrid = sphCompute.FindKernel("ClearGrid");
        kBuildGrid = sphCompute.FindKernel("BuildGrid");
        kDensityPressure = sphCompute.FindKernel("ComputeDensityPressure");
        kForces = sphCompute.FindKernel("ComputeForces");
        kIntegrate = sphCompute.FindKernel("Integrate");

        RecomputeTankLocalBoundsAndGrid();

        //Buffers:
        particleBuffer = new ComputeBuffer(particleCapacity, Marshal.SizeOf<ParticleData>());
        positionsBuffer = new ComputeBuffer(particleCapacity, sizeof(float) * 3);
        fluidInfoBuffer = new ComputeBuffer(particleCapacity, sizeof(float) * 2);

        cellCountsBuffer = new ComputeBuffer(cellCount, sizeof(uint));
        cellIndicesBuffer = new ComputeBuffer(cellCount * maxParticlesPerCell, sizeof(uint));

        boxesBuffer = new ComputeBuffer(4 * 64, sizeof(float) * 4);
        boxesBuffer.SetData(new Vector4[4 * 64]);

        //Fluids Table Buffer:
        fluidsBuffer = new ComputeBuffer(FluidLibrary.FluidCount, Marshal.SizeOf<FluidParamsGPU>());
        UploadFluidParamsTable();

        //Init Buffers:
        particleBuffer.SetData(new ParticleData[particleCapacity]);
        positionsBuffer.SetData(new Vector3[particleCapacity]);
        fluidInfoBuffer.SetData(new Vector2[particleCapacity]);
        cellCountsBuffer.SetData(new uint[cellCount]);
        cellIndicesBuffer.SetData(new uint[cellCount * maxParticlesPerCell]);

        //Bind All Kernels:
        BindCommon(kClearGrid);
        BindCommon(kBuildGrid);
        BindCommon(kDensityPressure);
        BindCommon(kForces);
        BindCommon(kIntegrate);

        //Constants:
        sphCompute.SetFloat(smoothingRadiusId, smoothingRadius);
        sphCompute.SetFloat(particleRadiusId, particleRadius);
        sphCompute.SetFloat(boundaryDampingId, boundaryDamping);

        sphCompute.SetInts(gridSizeId, gridX, gridY, gridZ);
        sphCompute.SetVector(gridOriginLocalId, tankGridOriginLocal);
        sphCompute.SetFloat(cellSizeId, smoothingRadius);
        sphCompute.SetInt(maxPerCellId, maxParticlesPerCell);

        sphCompute.SetVector(boundsMinLocalId, tankBoundsMinLocal);
        sphCompute.SetVector(boundsMaxLocalId, tankBoundsMaxLocal);

        //Kernel Constants (3D):
        float PI = Mathf.PI;
        float h = smoothingRadius;
        sphCompute.SetFloat(poly6ConstId, 315f / (64f * PI * Mathf.Pow(h, 9)));
        sphCompute.SetFloat(spikyConstId, -45f / (PI * Mathf.Pow(h, 6)));
        sphCompute.SetFloat(viscConstId, 45f / (PI * Mathf.Pow(h, 6)));

        //Rendering Binding:
        material.SetBuffer(positionsId, positionsBuffer);
        material.SetBuffer(fluidInfoId, fluidInfoBuffer);
        material.SetFloat(stepId, renderStep);

        //Start Empty:
        activeCount = 0;
        sphCompute.SetInt(particleCountId, activeCount);

        sphCompute.SetInt(boxCountId, 0);
        boxesTimer = 0f;

        if (enableSceneBoxColliders)
            UploadSceneBoxCollidersAsOBB_InTankLocal();
    }

    void OnDisable()
    {
        particleBuffer?.Release();
        positionsBuffer?.Release();
        fluidInfoBuffer?.Release();
        cellCountsBuffer?.Release();
        cellIndicesBuffer?.Release();
        boxesBuffer?.Release();
        fluidsBuffer?.Release();

        particleBuffer = null;
        positionsBuffer = null;
        fluidInfoBuffer = null;
        cellCountsBuffer = null;
        cellIndicesBuffer = null;
        boxesBuffer = null;
        fluidsBuffer = null;
    }

    void BindCommon(int kernel)
    {
        sphCompute.SetBuffer(kernel, particlesId, particleBuffer);
        sphCompute.SetBuffer(kernel, positionsId, positionsBuffer);
        sphCompute.SetBuffer(kernel, fluidInfoId, fluidInfoBuffer);
        sphCompute.SetBuffer(kernel, cellCountsId, cellCountsBuffer);
        sphCompute.SetBuffer(kernel, cellIndicesId, cellIndicesBuffer);

        sphCompute.SetBuffer(kernel, boxesId, boxesBuffer);
        sphCompute.SetBuffer(kernel, fluidsId, fluidsBuffer);
    }

    void UploadFluidParamsTable()
    {
        var arr = new FluidParamsGPU[FluidLibrary.FluidCount];
        for (int i = 0; i < FluidLibrary.FluidCount; i++)
        {
            var f = FluidLibrary.Get((FluidLibrary.FluidType)i);
            arr[i] = new FluidParamsGPU
            {
                restDensity = f.restDensity,
                gasConstant = f.gasConstant,
                viscosity = f.viscosity,
                pad = 0f
            };
        }
        fluidsBuffer.SetData(arr);
        sphCompute.SetInt(fluidCountId, FluidLibrary.FluidCount);
    }

    void Update()
    {
        if (spawnKeys == null) return;

        for (int i = 0; i < spawnKeys.Length; i++)
        {
            if (Input.GetKeyDown(spawnKeys[i].key))
                SpawnToGPU(spawnKeys[i].fluid);
        }
    }

    void FixedUpdate()
    {
        if (activeCount <= 0) return;

        if (enableSceneBoxColliders)
        {
            boxesTimer -= Time.fixedDeltaTime;
            if (boxesTimer <= 0f)
            {
                boxesTimer = sceneBoxesRefreshInterval;
                UploadSceneBoxCollidersAsOBB_InTankLocal();
            }
        }
        else
        {
            sphCompute.SetInt(boxCountId, 0);
        }

        //Update per-frame Parameters:
        sphCompute.SetFloat(timeStepId, timeStep);
        sphCompute.SetFloat(boundaryDampingId, boundaryDamping);
        sphCompute.SetFloat(particleRadiusId, particleRadius);

        Transform tankT = tankCollider.transform;
        sphCompute.SetVector(gravityLocalId, tankT.InverseTransformDirection(gravityWorld));
        sphCompute.SetMatrix(localToWorldId, tankT.localToWorldMatrix);

        sphCompute.SetInt(particleCountId, activeCount);

        int tgParticles = Mathf.CeilToInt(activeCount / 256f);
        if (tgParticles < 1) tgParticles = 1;

        int tgCells = Mathf.CeilToInt(cellCount / 256f);
        if (tgCells < 1) tgCells = 1;

        sphCompute.Dispatch(kClearGrid, tgCells, 1, 1);
        sphCompute.Dispatch(kBuildGrid, tgParticles, 1, 1);
        sphCompute.Dispatch(kDensityPressure, tgParticles, 1, 1);
        sphCompute.Dispatch(kForces, tgParticles, 1, 1);
        sphCompute.Dispatch(kIntegrate, tgParticles, 1, 1);
    }

    void LateUpdate()
    {
        if (activeCount <= 0) return;

        material.SetBuffer(positionsId, positionsBuffer);
        material.SetBuffer(fluidInfoId, fluidInfoBuffer);
        material.SetFloat(stepId, renderStep);

        Bounds wb = tankCollider.bounds;
        Bounds drawBounds = new Bounds(wb.center, wb.size + Vector3.one * renderBoundsMargin);

        Graphics.DrawMeshInstancedProcedural(mesh, 0, material, drawBounds, activeCount);
    }

    void RecomputeTankLocalBoundsAndGrid()
    {
        Vector3 half = tankCollider.size * 0.5f;
        Vector3 center = tankCollider.center;

        tankBoundsMinLocal = center - half;
        tankBoundsMaxLocal = center + half;
        tankGridOriginLocal = tankBoundsMinLocal;

        float cellSize = smoothingRadius;
        Vector3 size = tankBoundsMaxLocal - tankBoundsMinLocal;

        gridX = Mathf.Max(1, Mathf.CeilToInt(size.x / cellSize));
        gridY = Mathf.Max(1, Mathf.CeilToInt(size.y / cellSize));
        gridZ = Mathf.Max(1, Mathf.CeilToInt(size.z / cellSize));
        cellCount = Mathf.Max(1, gridX * gridY * gridZ);
    }

    void SpawnToGPU(FluidLibrary.FluidType fluidType)
    {
        if (particleBuffer == null) return;

        int writeOffset = appendSpawns ? activeCount : 0;
        int remaining = appendSpawns ? (particleCapacity - activeCount) : particleCapacity;
        if (remaining <= 0) return;

        float spacing = smoothingRadius * spawnSpacingMultiplier;

        float totalW = spawnWidth * spacing;
        float totalH = spawnHeight * spacing;
        float totalD = spawnDepth * spacing;

        Transform tankT = tankCollider.transform;

        Vector3 startWorld = spawnCenterWorld - new Vector3(totalW / 2f, totalH / 2f, totalD / 2f);

        float radius = Mathf.Min(totalW, Mathf.Min(totalH, totalD)) * 0.5f;
        float radiusSq = radius * radius;

        float jitter = spacing * 0.05f;

        //Particle Mass:
        var fs = FluidLibrary.Get(fluidType);
        float volume = spacing * spacing * spacing;
        float particleMass = fs.restDensity * volume;

        uint typeId = (uint)fluidType;

        var list = new List<ParticleData>(Mathf.Min(remaining, spawnWidth * spawnHeight * spawnDepth));

        for (int z = 0; z < spawnDepth && list.Count < remaining; z++)
            for (int y = 0; y < spawnHeight && list.Count < remaining; y++)
                for (int x = 0; x < spawnWidth && list.Count < remaining; x++)
                {
                    Vector3 pWorld = startWorld + new Vector3(x * spacing, y * spacing, z * spacing);

                    if (spawnAsSphere)
                    {
                        Vector3 local = pWorld - spawnCenterWorld;
                        if (local.sqrMagnitude > radiusSq) continue;
                    }

                    pWorld.x += Random.Range(-jitter, jitter);
                    pWorld.y += Random.Range(-jitter, jitter);
                    pWorld.z += Random.Range(-jitter, jitter);

                    Vector3 pLocal = tankT.InverseTransformPoint(pWorld);

                    if (pLocal.x < tankBoundsMinLocal.x || pLocal.x > tankBoundsMaxLocal.x ||
                        pLocal.y < tankBoundsMinLocal.y || pLocal.y > tankBoundsMaxLocal.y ||
                        pLocal.z < tankBoundsMinLocal.z || pLocal.z > tankBoundsMaxLocal.z)
                        continue;

                    list.Add(new ParticleData
                    {
                        pos = new Vector4(pLocal.x, pLocal.y, pLocal.z, 0f),
                        vel = Vector4.zero,
                        force = Vector4.zero,
                        density = fs.restDensity,
                        pressure = 0f,
                        typeId = typeId,
                        mass = particleMass,
                        pad = Vector4.zero
                    });
                }

        int spawned = list.Count;
        if (spawned <= 0) return;

        particleBuffer.SetData(list, 0, writeOffset, spawned);

        //Upload Positions and Fluid Info:
        Vector3[] posArr = new Vector3[spawned];
        Vector2[] infoArr = new Vector2[spawned];

        Matrix4x4 localToWorld = tankT.localToWorldMatrix;
        float typeNorm = (FluidLibrary.FluidCount > 1) ? ((float)typeId / (FluidLibrary.FluidCount - 1)) : 0f;

        for (int i = 0; i < spawned; i++)
        {
            Vector4 pl = list[i].pos;
            posArr[i] = localToWorld.MultiplyPoint3x4(new Vector3(pl.x, pl.y, pl.z));
            infoArr[i] = new Vector2(typeNorm, 1f); 
        }

        positionsBuffer.SetData(posArr, 0, writeOffset, spawned);
        fluidInfoBuffer.SetData(infoArr, 0, writeOffset, spawned);

        if (appendSpawns) activeCount += spawned;
        else activeCount = spawned;

        sphCompute.SetInt(particleCountId, activeCount);
    }

    void UploadSceneBoxCollidersAsOBB_InTankLocal()
    {
        var boxes = FindObjectsOfType<BoxCollider>()
            .Where(b => b != null && b.enabled && b != tankCollider &&
                        ((1 << b.gameObject.layer) & sceneBoxLayer.value) != 0)
            .ToArray();

        int count = boxes.Length;
        if (count <= 0)
        {
            sphCompute.SetInt(boxCountId, 0);
            return;
        }

        int neededFloat4 = count * 4;
        if (boxesBuffer.count < neededFloat4)
        {
            boxesBuffer.Release();
            boxesBuffer = new ComputeBuffer(Mathf.Max(neededFloat4, 4 * 64), sizeof(float) * 4);
            boxesBuffer.SetData(new Vector4[boxesBuffer.count]);

            BindCommon(kClearGrid);
            BindCommon(kBuildGrid);
            BindCommon(kDensityPressure);
            BindCommon(kForces);
            BindCommon(kIntegrate);
        }

        Transform tankT = tankCollider.transform;

        var packed = new Vector4[neededFloat4];
        for (int i = 0; i < count; i++)
        {
            BoxCollider bc = boxes[i];
            Transform t = bc.transform;

            Vector3 centerW = t.TransformPoint(bc.center);
            Vector3 axW = t.right.normalized;
            Vector3 ayW = t.up.normalized;
            Vector3 azW = t.forward.normalized;

            Vector3 lossy = t.lossyScale;
            Vector3 size = bc.size;
            Vector3 halfW = new Vector3(
                0.5f * size.x * Mathf.Abs(lossy.x),
                0.5f * size.y * Mathf.Abs(lossy.y),
                0.5f * size.z * Mathf.Abs(lossy.z)
            );

            Vector3 centerL = tankT.InverseTransformPoint(centerW);
            Vector3 axL = tankT.InverseTransformDirection(axW).normalized;
            Vector3 ayL = tankT.InverseTransformDirection(ayW).normalized;
            Vector3 azL = tankT.InverseTransformDirection(azW).normalized;

            int baseIdx = i * 4;
            packed[baseIdx + 0] = new Vector4(centerL.x, centerL.y, centerL.z, halfW.x);
            packed[baseIdx + 1] = new Vector4(axL.x, axL.y, axL.z, halfW.y);
            packed[baseIdx + 2] = new Vector4(ayL.x, ayL.y, ayL.z, halfW.z);
            packed[baseIdx + 3] = new Vector4(azL.x, azL.y, azL.z, 0f);
        }

        boxesBuffer.SetData(packed);
        sphCompute.SetInt(boxCountId, count);
    }

    void OnDrawGizmos()
    {
        float spacing = smoothingRadius * spawnSpacingMultiplier;
        float totalW = spawnWidth * spacing;
        float totalH = spawnHeight * spacing;
        float totalD = spawnDepth * spacing;
        float radius = Mathf.Min(totalW, Mathf.Min(totalH, totalD)) * 0.5f;

        Gizmos.color = Color.magenta;
        Gizmos.DrawWireSphere(spawnCenterWorld, radius);

        if (tankCollider != null)
        {
            Gizmos.color = Color.yellow;
            Transform t = tankCollider.transform;

            Vector3 centerW = t.TransformPoint(tankCollider.center);
            Vector3 lossy = t.lossyScale;
            Vector3 sizeW = new Vector3(
                tankCollider.size.x * Mathf.Abs(lossy.x),
                tankCollider.size.y * Mathf.Abs(lossy.y),
                tankCollider.size.z * Mathf.Abs(lossy.z)
            );

            Matrix4x4 old = Gizmos.matrix;
            Gizmos.matrix = Matrix4x4.TRS(centerW, t.rotation, Vector3.one);
            Gizmos.DrawWireCube(Vector3.zero, sizeW);
            Gizmos.matrix = old;
        }
    }
}
