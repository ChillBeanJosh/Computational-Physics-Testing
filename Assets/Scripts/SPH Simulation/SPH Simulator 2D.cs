using System.Collections.Generic;
using UnityEngine;

public class SPHSimulator2D : MonoBehaviour
{
    [Header("Particle Information")]
    public List<Particle> particles = new List<Particle>();
    public float globalMass = 1f;
    [Space]


    [Header("SPH Parameters")]
    public float restDensity = 1000f;
    public float gasConstant = 2000f;
    public float viscosityCoefficient = 1f;
    public float smoothingRadius = 0.2f;
    [Space]


    [Header("Simulation Parameters")]
    public Vector3 gravityValue = new Vector3(0, -9.81f, 0);
    public float timeStep = 0.006f;
    [Space]

    [Header("Boundary Settings")]
    [Range(0f, 100f)] public int gridWidth = 10;
    [Range(0f, 100f)] public int gridHeight = 10;
    public float gridSpacing = 0.5f;
    public float boundaryDamping = -0.5f;
    [Space]

    [Header("Spawner Settings")]
    public bool spawnAsCircle = true;
    [Space]
    public int spawnWidth = 10;
    public int spawnHeight = 20;
    public float spawnSpacingMultiplier = 1.0f;
    [Space]

    [Header("Debug Options")]
    public float densityRelaxRange = 50f;
    public bool debugDrawParticles = true;
    public bool debugColorByDensity = true;
    public bool debugDrawSmoothingRadius = true;
    [Space]

    [Header("References")]
    public KernelList Kernel;

    // Spatial grid
    private Dictionary<Vector2Int, List<Particle>> grid = new Dictionary<Vector2Int, List<Particle>>();


    private void Start()
    {
        float spacing = smoothingRadius * spawnSpacingMultiplier;

        //Calculate Global Mass:
        float area = spacing * spacing;
        globalMass = restDensity * area;
        

        GenerateParticleGrid(spacing);
    }

    private void FixedUpdate()
    {
        if (particles == null || particles.Count == 0) return;

        //Hashing Grid:
        BuildSpatialGrid();

        //SPH STEPS:
        ComputeParticleDensity();
        ComputeParticlePressure();

        //Reset Forces:
        foreach (var p in particles) p.force = Vector3.zero;

        //Calculate Forces:
        ComputePressureForce();
        ComputeViscosityForce();
        ComputeGravitationalForce();

        //Integrate Velocities and Positions:
        Integrate();

        //Boundary Conditions:
        CreateBoundaryConditions();
    }

    //----------------------------------------------------------------------------------------------------------------------------
    void ComputeParticleDensity()
    {
        foreach (var pi in particles)
        {
            float rho = 0f;
            var neighbors = GetNeighbors(pi);

            foreach (var pj in neighbors)
            {
                float r = (pi.position - pj.position).magnitude;
                rho += pj.Mass * Kernel.Poly6(r, smoothingRadius);
            }

            pi.density = Mathf.Max(rho, 1e-5f);
        }
    }

    void ComputeParticlePressure()
    {
        foreach (var p in particles)
        {
            float k = p.GasConstant;
            float rho0 = p.RestDensity;

            float pressure = k * (p.density - rho0);
            p.pressure = Mathf.Max(pressure, 0f);
        }
    }

    //----------------------------------------------------------------------------------------------------------------------------
    void ComputePressureForce()
    {
        foreach (var pi in particles)
        {
            Vector3 f = Vector3.zero;

            var neighbors = GetNeighbors(pi);

            foreach (var pj in neighbors)
            {
                if (pi == pj) continue;

                Vector3 rvec = pi.position - pj.position;
                float r = rvec.magnitude;

                if (r <= 0f || r >= smoothingRadius) continue;

                Vector3 gradW = Kernel.SpikyGradient(rvec, r, smoothingRadius);

                float term = (pi.pressure / (pi.density * pi.density)) + (pj.pressure / (pj.density * pj.density));

                f += -pj.Mass * term * gradW;
            }

            pi.force += f;
        }
    }

    void ComputeViscosityForce()
    {
        foreach (var pi in particles)
        {
            Vector3 f = Vector3.zero;
            var neighbors = GetNeighbors(pi);

            foreach (var pj in neighbors)
            {
                if (pi == pj) continue;

                float r = (pi.position - pj.position).magnitude;
                if (r >= smoothingRadius) continue;

                float lap = Kernel.ViscosityLaplacian(r, smoothingRadius);
                Vector3 dv = pj.velocity - pi.velocity;

                f += viscosityCoefficient * pj.Mass * (dv / pj.density) * lap;
            }

            pi.force += f;
        }
    }

    void ComputeGravitationalForce()
    {
        foreach (var p in particles)
        {
            p.force += p.Mass * gravityValue;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------

    void Integrate()
    {
        foreach (var p in particles)
        {
            Vector3 acceleration = p.force / p.density;
            p.velocity += acceleration * timeStep;
            p.position += p.velocity * timeStep;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------

    Vector2Int GetCell(Vector3 pos)
    {
        return new Vector2Int(
            Mathf.FloorToInt(pos.x / smoothingRadius),
            Mathf.FloorToInt(pos.y / smoothingRadius)
        );
    }

    void BuildSpatialGrid()
    {
        grid.Clear();
        foreach (var p in particles)
        {
            Vector2Int c = GetCell(p.position);
            if (!grid.ContainsKey(c))
                grid[c] = new List<Particle>();
            grid[c].Add(p);
        }
    }

    List<Particle> GetNeighbors(Particle p)
    {
        List<Particle> neighbors = new();
        Vector2Int c = GetCell(p.position);

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                Vector2Int nc = new Vector2Int(c.x + dx, c.y + dy);
                if (grid.ContainsKey(nc))
                    neighbors.AddRange(grid[nc]);
            }

        return neighbors;
    }

    //----------------------------------------------------------------------------------------------------------------------------
    void CreateBoundaryConditions()
    {
        float minX = 0f;
        float maxX = gridWidth * gridSpacing;

        float minY = 0f;
        float maxY = gridHeight * gridSpacing;

        foreach (var p in particles)
        {
            Vector3 pos = p.position;
            Vector3 vel = p.velocity;

            if (pos.x < minX) { pos.x = minX; vel.x *= boundaryDamping; }
            if (pos.x > maxX) { pos.x = maxX; vel.x *= boundaryDamping; }

            if (pos.y < minY) { pos.y = minY; vel.y *= boundaryDamping; }
            if (pos.y > maxY) { pos.y = maxY; vel.y *= boundaryDamping; }

            p.position = pos;
            p.velocity = vel;
        }
    }

    //----------------------------------------------------------------------------------------------------------------------------
    void GenerateParticleGrid(float spacing)
    {
        particles.Clear();

        Vector3 center = transform.position;
        float totalW = spawnWidth * spacing;
        float totalH = spawnHeight * spacing;

        Vector3 start = center - new Vector3(totalW / 2f, totalH / 2f, 0f);

        float radius = Mathf.Min(totalW, totalH) * 0.5f;
        float radiusSq = radius * radius;

        for (int y = 0; y < spawnHeight; y++)
        {
            for (int x = 0; x < spawnWidth; x++)
            {
                Vector3 pos = start + new Vector3(x * spacing, y * spacing, 0f);

                if (spawnAsCircle)
                {
                    Vector3 local = pos - center;
                    if (local.sqrMagnitude > radiusSq)
                        continue;
                }

                float jitter = spacing * 0.05f;
                pos.x += Random.Range(-jitter, jitter);
                pos.y += Random.Range(-jitter, jitter);

                Particle p = new Particle();

                p.Mass = globalMass;
                p.RestDensity = restDensity;
                p.GasConstant = gasConstant;
                p.ViscosityCoefficient = viscosityCoefficient;
                p.smoothingRadius = smoothingRadius;

                p.position = pos;
                p.velocity = Vector3.zero;
                p.force = Vector3.zero;

                p.density = restDensity;
                p.pressure = 0f;

                particles.Add(p);
            }
        }
    }

    //----------------------------------------------------------------------------------------------------------------------------
    Color FluidColorGradient(float t)
    {
        t = Mathf.Clamp01(t);

        //Colors: Blue → Cyan → Green → Yellow → Orange → Red
        Color[] cols =
        {
            new Color(0.0f, 0.4f, 1.0f), 
            Color.cyan,
            Color.green,
            Color.yellow,
            new Color(1f, 0.5f, 0f),    
            Color.red
        };

        //Scale t To Color Array:
        float scaled = t * (cols.Length - 1);
        int i = Mathf.FloorToInt(scaled);

        //Clamp Index:
        if (i >= cols.Length - 1)
            return cols[cols.Length - 1];

        //Interpolate:
        float localT = scaled - i;
        return Color.Lerp(cols[i], cols[i + 1], localT);
    }

    Color GetDensityColor(float density)
    {
        float rho0 = restDensity;

        //If within relaxed density range:
        //Blue:
        if (Mathf.Abs(density - rho0) <= densityRelaxRange) return new Color(0.0f, 0.4f, 1.0f);

        //------------------------------------------------------------

        //If under-density:
        if (density < rho0)
        {
            float t = Mathf.InverseLerp(rho0 - 3f * densityRelaxRange, rho0 - densityRelaxRange, density);

            //Blue to Cyan:
            return Color.Lerp
             (
                new Color(0.0f, 0.4f, 1.0f), // stable blue
                Color.cyan,
                t
             );
        }

        //------------------------------------------------------------

        //If over-density:
        float overT = Mathf.InverseLerp(rho0 + densityRelaxRange, rho0 + 6f * densityRelaxRange, density);

        //Colors: Cyan → Green → Yellow → Orange → Red
        Color[] cols =
        {
            new Color(0.0f, 0.4f, 1.0f), 
            Color.green,
            Color.yellow,
            new Color(1f, 0.5f, 0f),     
            Color.red
        };

        //Scale t To Color Array:
        float scaled = overT * (cols.Length - 1);
        int i = Mathf.FloorToInt(scaled);

        //Clamp Index:
        if (i >= cols.Length - 1)
            return cols[^1];

        //Interpolate:
        float localT = scaled - i;
        return Color.Lerp(cols[i], cols[i + 1], localT);
    }

    private void OnDrawGizmos()
    {
        //Draw Tank Boundary:
        float tankW = gridWidth * gridSpacing;
        float tankH = gridHeight * gridSpacing;

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube
        (
            new Vector3(tankW * 0.5f, tankH * 0.5f, 0f),
            new Vector3(tankW, tankH, 0.01f)
        );

        //------------------------------------------------------------

        //Draw Spawner Area:
        float spacing = smoothingRadius * spawnSpacingMultiplier;
        float spawnW = spawnWidth * spacing;
        float spawnH = spawnHeight * spacing;

        Gizmos.color = Color.magenta;
        Gizmos.DrawWireCube
        (
            transform.position,
            new Vector3(spawnW, spawnH, 0.01f)
        );

        //------------------------------------------------------------

        if (!Application.isPlaying || particles == null) return;

        //Draw Particles:
        foreach (var p in particles)
        {
            //Particle Color By Density:
            if (debugColorByDensity)
                Gizmos.color = GetDensityColor(p.density);
            else
                Gizmos.color = Color.white;

            //Draw Particle:
            Gizmos.DrawSphere(p.position, 0.05f);

            //Draw Smoothing Radius:
            if (debugDrawSmoothingRadius)
            {
                Gizmos.color = new Color(0f, 1f, 1f, 0.15f);
                Gizmos.DrawWireSphere(p.position, smoothingRadius);
            }
        }
    }
}


