using UnityEngine;

public class GPUSPH2D : MonoBehaviour
{
    [Header("Particle Count")]
    [SerializeField, Range(256, 200000)] int particleCount = 20000;

    [Header("SPH Parameters")]
    [SerializeField] float restDensity = 1000f;
    [SerializeField] float gasConstant = 2000f;
    [SerializeField] float viscosity = 1f;
    [SerializeField] float smoothingRadius = 0.2f;

    [Header("Simulation")]
    [SerializeField] Vector2 gravity = new Vector2(0, -9.81f);
    [SerializeField] float timeStep = 0.006f;
    [SerializeField] float boundaryDamping = -0.5f;

    [Header("Tank Bounds")]
    [SerializeField] Vector2 boundsMin = new Vector2(0, 0);
    [SerializeField] Vector2 boundsMax = new Vector2(10, 10);

    [Header("GPU Grid")]
    [SerializeField] int maxParticlesPerCell = 64; // raise if dense clumping happens

    [Header("Spawn")]
    [SerializeField] bool spawnAsCircle = true;
    [SerializeField] int spawnWidth = 200;
    [SerializeField] int spawnHeight = 100;
    [SerializeField] float spawnSpacingMultiplier = 1.0f;

    [Header("Rendering (same as GPUGraph)")]
    [SerializeField] Material material;
    [SerializeField] Mesh mesh;
    [SerializeField] float renderStep = 0.05f; // point size scale (maps to _Step)

    [Header("Compute")]
    [SerializeField] ComputeShader sphCompute;

    ComputeBuffer particleBuffer;
    ComputeBuffer positionsBuffer;
    ComputeBuffer cellCountsBuffer;
    ComputeBuffer cellIndicesBuffer;

    // Kernels
    int kClearGrid, kBuildGrid, kDensityPressure, kForces, kIntegrate;

    // IDs (match compute shader)
    static readonly int
        particleCountId = Shader.PropertyToID("_ParticleCount"),
        timeStepId = Shader.PropertyToID("_TimeStep"),
        gravityId = Shader.PropertyToID("_Gravity"),
        restDensityId = Shader.PropertyToID("_RestDensity"),
        gasConstantId = Shader.PropertyToID("_GasConstant"),
        viscosityId = Shader.PropertyToID("_Viscosity"),
        smoothingRadiusId = Shader.PropertyToID("_SmoothingRadius"),
        particleMassId = Shader.PropertyToID("_ParticleMass"),

        boundsMinId = Shader.PropertyToID("_BoundsMin"),
        boundsMaxId = Shader.PropertyToID("_BoundsMax"),
        boundaryDampingId = Shader.PropertyToID("_BoundaryDamping"),

        gridSizeId = Shader.PropertyToID("_GridSize"),
        gridOriginId = Shader.PropertyToID("_GridOrigin"),
        cellSizeId = Shader.PropertyToID("_CellSize"),
        maxPerCellId = Shader.PropertyToID("_MaxParticlesPerCell"),

        poly6ConstId = Shader.PropertyToID("_Poly6Constant"),
        spikyConstId = Shader.PropertyToID("_SpikyGradConstant"),
        viscConstId = Shader.PropertyToID("_ViscLapConstant"),

        particlesId = Shader.PropertyToID("_Particles"),
        positionsId = Shader.PropertyToID("_Positions"),
        cellCountsId = Shader.PropertyToID("_CellCounts"),
        cellIndicesId = Shader.PropertyToID("_CellIndices"),

        stepId = Shader.PropertyToID("_Step");

    struct ParticleData
    {
        public Vector4 pos;
        public Vector4 vel;
        public Vector4 force;
        public float density;
        public float pressure;
        public Vector2 pad;
    }

    void OnEnable()
    {
        // Kernels
        kClearGrid = sphCompute.FindKernel("ClearGrid");
        kBuildGrid = sphCompute.FindKernel("BuildGrid");
        kDensityPressure = sphCompute.FindKernel("ComputeDensityPressure");
        kForces = sphCompute.FindKernel("ComputeForces");
        kIntegrate = sphCompute.FindKernel("Integrate");

        // Grid sizing
        float cellSize = smoothingRadius;
        int gridX = Mathf.CeilToInt((boundsMax.x - boundsMin.x) / cellSize);
        int gridY = Mathf.CeilToInt((boundsMax.y - boundsMin.y) / cellSize);
        int cellCount = Mathf.Max(1, gridX * gridY);

        // Mass: same idea as your CPU start()
        float spacing = smoothingRadius * spawnSpacingMultiplier;
        float area = spacing * spacing;
        float particleMass = restDensity * area;

        // Buffers
        particleBuffer = new ComputeBuffer(particleCount, System.Runtime.InteropServices.Marshal.SizeOf<ParticleData>());
        positionsBuffer = new ComputeBuffer(particleCount, sizeof(float) * 3);

        cellCountsBuffer = new ComputeBuffer(cellCount, sizeof(uint));
        cellIndicesBuffer = new ComputeBuffer(cellCount * maxParticlesPerCell, sizeof(uint));

        // Init particles
        var data = new ParticleData[particleCount];
        InitSpawn(data, spacing);
        particleBuffer.SetData(data);

        // Initialize positions buffer from particle data so first-frame render is correct
        var posArray = new Vector3[particleCount];
        for (int i = 0; i < particleCount; i++)
        {
            posArray[i] = new Vector3(data[i].pos.x, data[i].pos.y, 0f);
        }
        positionsBuffer.SetData(posArray);

        // Clear grid buffers on the CPU before first dispatch
        var zeroCounts = new uint[cellCount];
        cellCountsBuffer.SetData(zeroCounts);
        var zeroIndices = new uint[cellCount * maxParticlesPerCell];
        cellIndicesBuffer.SetData(zeroIndices);

        // Bind buffers to all kernels that need them
        BindCommon(kClearGrid);
        BindCommon(kBuildGrid);
        BindCommon(kDensityPressure);
        BindCommon(kForces);
        BindCommon(kIntegrate);

        // Set constants (once)
        sphCompute.SetInt(particleCountId, particleCount);

        sphCompute.SetFloat(restDensityId, restDensity);
        sphCompute.SetFloat(gasConstantId, gasConstant);
        sphCompute.SetFloat(viscosityId, viscosity);
        sphCompute.SetFloat(smoothingRadiusId, smoothingRadius);
        sphCompute.SetFloat(particleMassId, particleMass);

        sphCompute.SetVector(boundsMinId, boundsMin);
        sphCompute.SetVector(boundsMaxId, boundsMax);
        sphCompute.SetFloat(boundaryDampingId, boundaryDamping);

        sphCompute.SetVector(gridOriginId, boundsMin);
        // use SetInts to map properly to uint2 in shader
        sphCompute.SetInts(gridSizeId, gridX, gridY);
        sphCompute.SetFloat(cellSizeId, cellSize);
        sphCompute.SetInt(maxPerCellId, maxParticlesPerCell);

        // Kernel constants
        float PI = Mathf.PI;
        float h = smoothingRadius;

        sphCompute.SetFloat(poly6ConstId, 4f / (PI * Mathf.Pow(h, 8)));
        sphCompute.SetFloat(spikyConstId, -30f / (PI * Mathf.Pow(h, 5)));
        sphCompute.SetFloat(viscConstId, 40f / (PI * Mathf.Pow(h, 5)));

        // Material binding (like GPUGraph)
        material.SetBuffer(positionsId, positionsBuffer);
        material.SetFloat(stepId, renderStep);
    }

    void OnDisable()
    {
        particleBuffer?.Release();
        positionsBuffer?.Release();
        cellCountsBuffer?.Release();
        cellIndicesBuffer?.Release();

        particleBuffer = null;
        positionsBuffer = null;
        cellCountsBuffer = null;
        cellIndicesBuffer = null;
    }

    void BindCommon(int kernel)
    {
        sphCompute.SetBuffer(kernel, particlesId, particleBuffer);
        sphCompute.SetBuffer(kernel, positionsId, positionsBuffer);
        sphCompute.SetBuffer(kernel, cellCountsId, cellCountsBuffer);
        sphCompute.SetBuffer(kernel, cellIndicesId, cellIndicesBuffer);
    }

    void FixedUpdate()
    {
        // Per-step params
        sphCompute.SetFloat(timeStepId, timeStep);
        sphCompute.SetVector(gravityId, gravity);

        // Dispatch sizes
        int threadGroupsParticles = Mathf.CeilToInt((float)particleCount / 256f);

        int gridX = Mathf.CeilToInt((boundsMax.x - boundsMin.x) / smoothingRadius);
        int gridY = Mathf.CeilToInt((boundsMax.y - boundsMin.y) / smoothingRadius);
        int cellCount = Mathf.Max(1, gridX * gridY);
        int threadGroupsCells = Mathf.CeilToInt((float)cellCount / 256f);

        // Pipeline
        sphCompute.Dispatch(kClearGrid, threadGroupsCells, 1, 1);
        sphCompute.Dispatch(kBuildGrid, threadGroupsParticles, 1, 1);
        sphCompute.Dispatch(kDensityPressure, threadGroupsParticles, 1, 1);
        sphCompute.Dispatch(kForces, threadGroupsParticles, 1, 1);
        sphCompute.Dispatch(kIntegrate, threadGroupsParticles, 1, 1);

        // Draw procedural instanced (like GPUGraph)
        var size = new Vector3(boundsMax.x - boundsMin.x, boundsMax.y - boundsMin.y, 1f);
        var center = new Vector3((boundsMin.x + boundsMax.x) * 0.5f, (boundsMin.y + boundsMax.y) * 0.5f, 0f);
        var bounds = new Bounds(center, size + Vector3.one * 2f);

        Graphics.DrawMeshInstancedProcedural(mesh, 0, material, bounds, particleCount);
    }

    void InitSpawn(ParticleData[] data, float spacing)
    {
        // Mirror CPU GenerateParticleGrid: center-based spawn area with optional circular mask
        int idx = 0;
        Vector3 center = transform.position;
        float totalW = spawnWidth * spacing;
        float totalH = spawnHeight * spacing;
        Vector3 start = center - new Vector3(totalW / 2f, totalH / 2f, 0f);

        float radius = Mathf.Min(totalW, totalH) * 0.5f;
        float radiusSq = radius * radius;

        for (int y = 0; y < spawnHeight && idx < data.Length; y++)
        {
            for (int x = 0; x < spawnWidth && idx < data.Length; x++)
            {
                Vector3 pos = start + new Vector3(x * spacing, y * spacing, 0f);

                if (spawnAsCircle)
                {
                    Vector3 local = pos - center;
                    if (local.sqrMagnitude > radiusSq)
                        continue;
                }

                // jitter like CPU
                float jitter = spacing * 0.05f;
                pos.x += Random.Range(-jitter, jitter);
                pos.y += Random.Range(-jitter, jitter);

                data[idx] = new ParticleData
                {
                    pos = new Vector4(pos.x, pos.y, 0f, 0f),
                    vel = Vector4.zero,
                    force = Vector4.zero,
                    density = restDensity,
                    pressure = 0f,
                    pad = Vector2.zero
                };
                idx++;
            }
        }

        // If spawnWidth*spawnHeight < particleCount, remaining particles stay at origin (you can scatter if desired)
    }
}
