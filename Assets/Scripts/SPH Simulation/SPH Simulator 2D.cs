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

    [Header("Boundary / Collision")]
    public float boundaryDamping = 0.5f; // use positive restitution multiplier
    public LayerMask collisionMask = ~0; // default: all layers
    [Space]

    [Header("Grid (for neighbor search)")]
    public float gridSpacing = 0.5f;
    [Space]

    [Header("Spawner Settings")]
    public bool spawnAsSphere = true;
    [Space]
    public int spawnWidth = 10;
    public int spawnHeight = 10;
    public int spawnDepth = 10;
    public float spawnSpacingMultiplier = 1.0f;
    public KeyCode spawnKey = KeyCode.Space;
    public Vector3 spawnCenter = Vector3.zero; // world position to spawn around
    [Space]

    [Header("Debug Options")]
    public float densityRelaxRange = 50f;
    public bool debugDrawParticles = true;
    public bool debugColorByDensity = true;
    public bool debugDrawSmoothingRadius = true;
    [Space]

    [Header("References")]
    public KernelList Kernel;

    // Spatial grid (3D)
    private Dictionary<Vector3Int, List<Particle>> grid = new Dictionary<Vector3Int, List<Particle>>();

    private void Start()
    {
        // No particles created at start per request.
    }

    private void Update()
    {
        if (Input.GetKeyDown(spawnKey))
        {
            float spacing = smoothingRadius * spawnSpacingMultiplier;
            SpawnParticles(spawnCenter, spacing);
        }
    }

    private void FixedUpdate()
    {
        if (particles == null || particles.Count == 0) return;

        // Hashing Grid:
        BuildSpatialGrid();

        // SPH STEPS:
        ComputeParticleDensity();
        ComputeParticlePressure();

        // Reset Forces:
        foreach (var p in particles) p.force = Vector3.zero;

        // Calculate Forces:
        ComputePressureForce();
        ComputeViscosityForce();
        ComputeGravitationalForce();

        // Integrate Velocities and Positions:
        Integrate();

        // Collisions against scene colliders (rigidbodies / static colliders)
        HandleCollisions();
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

    Vector3Int GetCell(Vector3 pos)
    {
        return new Vector3Int(
            Mathf.FloorToInt(pos.x / smoothingRadius),
            Mathf.FloorToInt(pos.y / smoothingRadius),
            Mathf.FloorToInt(pos.z / smoothingRadius)
        );
    }

    void BuildSpatialGrid()
    {
        grid.Clear();
        foreach (var p in particles)
        {
            Vector3Int c = GetCell(p.position);
            if (!grid.ContainsKey(c))
                grid[c] = new List<Particle>();
            grid[c].Add(p);
        }
    }

    List<Particle> GetNeighbors(Particle p)
    {
        List<Particle> neighbors = new();
        Vector3Int c = GetCell(p.position);

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    Vector3Int nc = new Vector3Int(c.x + dx, c.y + dy, c.z + dz);
                    if (grid.ContainsKey(nc))
                        neighbors.AddRange(grid[nc]);
                }

        return neighbors;
    }

    //----------------------------------------------------------------------------------------------------------------------------
    void HandleCollisions()
    {
        float collisionRadius = Mathf.Max(0.01f, smoothingRadius * 0.5f);

        for (int i = 0; i < particles.Count; i++)
        {
            var p = particles[i];
            Collider[] cols = Physics.OverlapSphere(p.position, collisionRadius, collisionMask, QueryTriggerInteraction.Ignore);
            if (cols == null || cols.Length == 0) continue;

            foreach (var col in cols)
            {
                if (col == null) continue;

                Vector3 closest = col.ClosestPoint(p.position);
                Vector3 dir = p.position - closest;
                float dist = dir.magnitude;

                // If inside collider (or too close), push out and damp velocity
                if (dist < 1e-4f)
                {
                    // fallback normal from collider transform
                    dir = (p.position - col.transform.position);
                    if (dir.sqrMagnitude < 1e-6f) dir = Vector3.up;
                    dir = dir.normalized;
                    p.position = closest + dir * collisionRadius;
                }
                else if (dist < collisionRadius)
                {
                    Vector3 normal = dir.normalized;
                    p.position = closest + normal * collisionRadius;
                    p.velocity = Vector3.Reflect(p.velocity, normal) * boundaryDamping;
                }

                // update particle in list
                particles[i] = p;
            }
        }
    }

    //----------------------------------------------------------------------------------------------------------------------------
    void SpawnParticles(Vector3 center, float spacing)
    {
        float totalW = spawnWidth * spacing;
        float totalH = spawnHeight * spacing;
        float totalD = spawnDepth * spacing;

        Vector3 start = center - new Vector3(totalW / 2f, totalH / 2f, totalD / 2f);

        float radius = Mathf.Min(totalW, Mathf.Min(totalH, totalD)) * 0.5f;
        float radiusSq = radius * radius;

        // Compute per-particle mass (volume for 3D)
        float volume = spacing * spacing * spacing;
        float particleMass = restDensity * volume;

        for (int z = 0; z < spawnDepth; z++)
        {
            for (int y = 0; y < spawnHeight; y++)
            {
                for (int x = 0; x < spawnWidth; x++)
                {
                    Vector3 pos = start + new Vector3(x * spacing, y * spacing, z * spacing);

                    if (spawnAsSphere)
                    {
                        Vector3 local = pos - center;
                        if (local.sqrMagnitude > radiusSq)
                            continue;
                    }

                    float jitter = spacing * 0.05f;
                    pos.x += Random.Range(-jitter, jitter);
                    pos.y += Random.Range(-jitter, jitter);
                    pos.z += Random.Range(-jitter, jitter);

                    Particle p = new Particle();

                    p.Mass = particleMass;
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
        if (Mathf.Abs(density - rho0) <= densityRelaxRange) return new Color(0.0f, 0.4f, 1.0f);

        //------------------------------------------------------------
        if (density < rho0)
        {
            float t = Mathf.InverseLerp(rho0 - 3f * densityRelaxRange, rho0 - densityRelaxRange, density);

            //Blue to Cyan:
            return Color.Lerp
             (
                new Color(0.0f, 0.4f, 1.0f),
                Color.cyan,
                t
             );
        }

        //------------------------------------------------------------
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

        float scaled = overT * (cols.Length - 1);
        int idx = Mathf.FloorToInt(scaled);

        if (idx >= cols.Length - 1)
            return cols[^1];

        float local = scaled - idx;
        return Color.Lerp(cols[idx], cols[idx + 1], local);
    }

    private void OnDrawGizmos()
    {
        // Draw Spawner Area:
        float spacing = smoothingRadius * spawnSpacingMultiplier;
        float spawnW = spawnWidth * spacing;
        float spawnH = spawnHeight * spacing;
        float spawnD = spawnDepth * spacing;

        Gizmos.color = Color.magenta;
        Gizmos.DrawWireCube(
            spawnCenter,
            new Vector3(spawnW, spawnH, spawnD)
        );

        if (!Application.isPlaying || particles == null) return;

        // Draw Particles:
        foreach (var p in particles)
        {
            if (debugColorByDensity)
                Gizmos.color = GetDensityColor(p.density);
            else
                Gizmos.color = Color.white;

            Gizmos.DrawSphere(p.position, Mathf.Min(0.05f, smoothingRadius * 0.2f));

            if (debugDrawSmoothingRadius)
            {
                Gizmos.color = new Color(0f, 1f, 1f, 0.15f);
                Gizmos.DrawWireSphere(p.position, smoothingRadius);
            }
        }
    }
}


