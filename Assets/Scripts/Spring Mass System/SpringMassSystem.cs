using System.Collections.Generic;
using UnityEngine;
using static UnityEditor.Experimental.GraphView.GraphView;

public class SpringMassSystem : MonoBehaviour
{
    [Header("Mass & Spring Data")]
    public List<MassPoint> masses = new List<MassPoint>();
    public List<SpringElement> springs = new List<SpringElement>();
    [Space]

    [Header("Simulation Parameters")]
    public float globalMassValue;
    [Space]
    public int gridWidth;
    public int gridHeight;
    public float gridSpacing;
    [Space]
    [Range(0f, 2f)] public float dampingRatio;
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public float timeStep;
    [Space]

    [Header("Collision Parameters")]
    public LayerMask collisionLayerMask;
    public float collisionRadius;
    [Range(0f, 1f)] public float restitution;      // Bounce factor
    [Range(0f, 1f)] public float friction;         // Friction factor
    [Space]

    [Header("SPH Parameters")]
    public float restDensity;   // ρ0
    public float pressureStiffness; // k in P = k(ρ-ρ0)
    [Range(0f, 1f)] public float viscosity;      // μ
    public float globalSmoothingRadius; // h (used by all particles)
    [Space]

    [Header("Boundary Particle Parameters")]
    public int BoundaryLayers;
    public float BoundaryMassMultiplier;
    public float BoundaryDensityMultiplier;
    [Space]

    [Header("Background Heatmap")]
    public bool debugDrawBackground = true;
    [Space]
    public Color backgroundLowColor;
    public Color backgroundMidColor;
    public Color backgroundHighColor;
    [Space]
    [Range(10, 200)] public int backgroundResolution = 60;   // number of samples horizontally
    [Space]

    [Header("Springs Toggle")]
    public bool useSprings = true;
    [Space]

    [Header("Debug")]
    public bool showDebug = true;
    [Space]
    public bool debugColorByDensity = true;
    public bool debugDrawSmoothingRadius = true;
    public bool debugDrawSPHForces = true;
    [Space]
    [Range(0.0005f, 0.002f)] public float debugForceScale;
    public float debugMinDensity;
    public float debugMaxDensity;

    void Start()
    {
        if (gridWidth > 0 && gridHeight > 0)
        {
            GenerateMassGrid();
        }

        foreach (var m in masses)
        {
            m.mass = globalMassValue;
            m.smoothingRadius = globalSmoothingRadius;

            m.isFixed = false;

            m.isBoundary = false;
            m.boundaryVolume = 0f;
        }

        GenerateBoundaryParticles();

        PrecomputeBoundaryVolumes();

        InitializeRestLengths();
    }

    void FixedUpdate()
    {
        //1.) SPH Density & Pressure:
        ComputeDensities();
        ComputePressures();

        //2.) External Forces (Gravity):
        ApplyExternalForces();

        //3.) Spring Forces:
        if (useSprings) ApplySpringForces();

        //4.) SPH Internal Forces:
        ApplySPHPressureForces();
        ApplySPHViscosityForces();

        //5.) Integrate Motion:
        Integrate();
    }

    // ----------------------------------------------------------------------
    void GenerateMassGrid()
    {
        masses.Clear();
        springs.Clear();

        int count = gridWidth * gridHeight;
        masses = new List<MassPoint>(count);

        // 1) Create masses
        for (int y = 0; y < gridHeight; y++)
        {
            for (int x = 0; x < gridWidth; x++)
            {
                MassPoint m = new MassPoint
                {
                    mass = globalMassValue,
                    density = restDensity,
                    pressure = 0,
                    position = new Vector3(x * gridSpacing, y * gridSpacing, 0f),
                    velocity = Vector3.zero,
                    force = Vector3.zero,
                    sphForce = Vector3.zero,
                    smoothingRadius = globalSmoothingRadius,
                    isFixed = false,
                    isBoundary = false,
                    boundaryVolume = 0f
                };

                masses.Add(m);
            }
        }

        // 2) Connect grid with springs if spring simulation is toggled on
        if (useSprings)
        {
            for (int y = 0; y < gridHeight; y++)
            {
                for (int x = 0; x < gridWidth; x++)
                {
                    int i = y * gridWidth + x;

                    // Horizontal spring
                    if (x < gridWidth - 1)
                        springs.Add(new SpringElement
                        {
                            startMassIndex = i,
                            endMassIndex = i + 1,
                            stiffness = 50f,
                            damping = 1f
                        });

                    // Vertical spring
                    if (y < gridHeight - 1)
                        springs.Add(new SpringElement
                        {
                            startMassIndex = i,
                            endMassIndex = i + gridWidth,
                            stiffness = 50f,
                            damping = 1f
                        });

                    // Optional diagonals for a stiffer grid
                    if (x < gridWidth - 1 && y < gridHeight - 1)
                    {
                        springs.Add(new SpringElement
                        {
                            startMassIndex = i,
                            endMassIndex = i + gridWidth + 1,
                            stiffness = 50f,
                            damping = 1f
                        });

                        springs.Add(new SpringElement
                        {
                            startMassIndex = i + 1,
                            endMassIndex = i + gridWidth,
                            stiffness = 50f,
                            damping = 1f
                        });
                    }
                }
            }
        }
    }

    // ----------------------------------------------------------------------
    void GenerateBoundaryParticles()
    {
        float h = globalSmoothingRadius;
        float spacing = h * 0.3f;

        // Determine fluid bounds from generated grid
        float minX = 0f;
        float maxX = (gridWidth - 1) * gridSpacing;

        float minY = 0f;
        float maxY = (gridHeight - 1) * gridSpacing;

        List<MassPoint> boundary = new List<MassPoint>();

        // TOP WALL
        for (int l = 0; l < BoundaryLayers; l++)
        {
            float y = maxY + (l + 1) * spacing;
            for (float x = minX - spacing; x <= maxX + spacing; x += spacing)
            {
                boundary.Add(NewBoundaryParticle(new Vector3(x, y, 0)));
            }
        }

        // BOTTOM WALL
        for (int l = 0; l < BoundaryLayers; l++)
        {
            float y = minY - (l + 1) * spacing;
            for (float x = minX - spacing; x <= maxX + spacing; x += spacing)
            {
                boundary.Add(NewBoundaryParticle(new Vector3(x, y, 0)));
            }
        }

        // LEFT WALL
        for (int l = 0; l < BoundaryLayers; l++)
        {
            float x = minX - (l + 1) * spacing;
            for (float y = minY - spacing; y <= maxY + spacing; y += spacing)
            {
                boundary.Add(NewBoundaryParticle(new Vector3(x, y, 0)));
            }
        }

        // RIGHT WALL
        for (int l = 0; l < BoundaryLayers; l++)
        {
            float x = maxX + (l + 1) * spacing;
            for (float y = minY - spacing; y <= maxY + spacing; y += spacing)
            {
                boundary.Add(NewBoundaryParticle(new Vector3(x, y, 0)));
            }
        }

        // Add to system
        masses.AddRange(boundary);
    }

    MassPoint NewBoundaryParticle(Vector3 pos)
    {
        return new MassPoint
        {
            position = pos,
            velocity = Vector3.zero,

            force = Vector3.zero,
            sphForce = Vector3.zero,

            mass = globalMassValue * BoundaryMassMultiplier,

            density = restDensity * BoundaryDensityMultiplier,
            pressure = 0f,

            smoothingRadius = globalSmoothingRadius,

            isFixed = true,
            isBoundary = true,
            boundaryVolume = 0f
        };
    }

    void PrecomputeBoundaryVolumes()
    {
        float h = globalSmoothingRadius;

        // Akinci: V_b = 1 / Σ_k W(|x_b - x_k|), sum over boundary neighbors
        for (int i = 0; i < masses.Count; i++)
        {
            MassPoint bi = masses[i];
            if (!bi.isBoundary) continue;

            float sumW = 0f;

            for (int j = 0; j < masses.Count; j++)
            {
                MassPoint bj = masses[j];
                if (!bj.isBoundary) continue;

                float r = (bi.position - bj.position).magnitude;
                sumW += Poly6Kernel(r, h);
            }

            bi.boundaryVolume = (sumW > 1e-6f) ? 1f / sumW : 0f;
            masses[i] = bi;
        }
    }

    // ----------------------------------------------------------------------
    void InitializeRestLengths()
    {
        foreach (var s in springs)
        {
            if (s.startMassIndex < 0 || s.startMassIndex >= masses.Count ||
                s.endMassIndex < 0 || s.endMassIndex >= masses.Count)
            {
                continue;
            }

            Vector3 pA = masses[s.startMassIndex].position;
            Vector3 pB = masses[s.endMassIndex].position;
            s.restLength = Vector3.Distance(pA, pB);
        }
    }

    // ----------------------------------------------------------------------
    void ApplyExternalForces()
    {
        for (int i = 0; i < masses.Count; i++)
        {
            var m = masses[i];

            if (m.isFixed)
            {
                m.force = Vector3.zero;
                m.sphForce = Vector3.zero;
            }
            else
            {
                m.force = m.mass * gravity;
                m.sphForce = Vector3.zero;
            }

            masses[i] = m;
        }
    }

    // ----------------------------------------------------------------------
    void ApplySpringForces()
    {
        foreach (var s in springs)
        {
            if (s.startMassIndex < 0 || s.startMassIndex >= masses.Count ||
                s.endMassIndex < 0 || s.endMassIndex >= masses.Count)
            {
                continue;
            }

            MassPoint A = masses[s.startMassIndex];
            MassPoint B = masses[s.endMassIndex];

            Vector3 delta = B.position - A.position;
            float dist = delta.magnitude;
            if (dist <= 1e-6f) continue;

            Vector3 dir = delta / dist;
            float x = dist - s.restLength;

            // Hooke
            Vector3 springForce = s.stiffness * x * dir;

            // Damping (along spring direction)
            Vector3 relVel = B.velocity - A.velocity;
            Vector3 dampingForce = s.damping * Vector3.Dot(relVel, dir) * dir;

            Vector3 total = springForce + dampingForce;

            if (!A.isFixed) A.force += total;
            if (!B.isFixed) B.force -= total;

            masses[s.startMassIndex] = A;
            masses[s.endMassIndex] = B;
        }
    }

    // ----------------------------------------------------------------------
    // SPH Kernels
    float Poly6Kernel(float r, float h)
    {
        if (r < 0f || r > h) return 0f;
        float h2 = h * h;
        float r2 = r * r;
        float term = h2 - r2;
        float coeff = 315f / (64f * Mathf.PI * Mathf.Pow(h, 9));
        return coeff * term * term * term;
    }

    Vector3 SpikyGradient(Vector3 rVec, float h)
    {
        float r = rVec.magnitude;
        if (r < 1e-6f || r > h) return Vector3.zero;

        float coeff = -45f / (Mathf.PI * Mathf.Pow(h, 6));
        float term = (h - r) * (h - r);
        float scalar = coeff * term / r;
        return scalar * rVec; // gradient wrt particle position
    }

    float ViscosityLaplacian(float r, float h)
    {
        if (r < 0f || r > h) return 0f;
        float coeff = 45f / (Mathf.PI * Mathf.Pow(h, 6));
        return coeff * (h - r);
    }

    // ----------------------------------------------------------------------
    void ComputeDensities()
    {
        float h = globalSmoothingRadius;
        int n = masses.Count;

        for (int i = 0; i < n; i++)
        {
            MassPoint pi = masses[i];

            // Boundary particles just keep rest density
            if (pi.isBoundary)
            {
                pi.density = restDensity;
                masses[i] = pi;
                continue;
            }

            float density = 0f;

            for (int j = 0; j < n; j++)
            {
                MassPoint pj = masses[j];
                float r = (pi.position - pj.position).magnitude;
                float w = Poly6Kernel(r, h);

                if (!pj.isBoundary)
                {
                    density += pj.mass * w; // fluid contribution
                }
                else
                {
                    // Akinci boundary contribution: ρ0 * V_b * W
                    density += restDensity * pj.boundaryVolume * w;
                }
            }

            pi.density = Mathf.Max(density, 1e-6f); // avoid divide-by-zero
            masses[i] = pi;
        }
    }

    // ----------------------------------------------------------------------
    void ComputePressures()
    {
         for (int i = 0; i < masses.Count; i++)
        {
            var p = masses[i];

            if (p.isBoundary)
            {
                p.pressure = 0f; // boundary pressure handled implicitly
            }
            else
            {
                p.pressure = pressureStiffness * (p.density - restDensity);
            }

            masses[i] = p;
        }
    }

    // ----------------------------------------------------------------------
    void ApplySPHPressureForces()
    {
        float h = globalSmoothingRadius;
        int n = masses.Count;

        // Precompute ρ0^2 for boundary term
        float rho0Sq = restDensity * restDensity;

        for (int i = 0; i < n; i++)
        {
            MassPoint pi = masses[i];
            if (pi.isBoundary) continue;

            Vector3 pressureForce = Vector3.zero;

            for (int j = 0; j < n; j++)
            {
                if (i == j) continue;

                MassPoint pj = masses[j];

                Vector3 diff = pi.position - pj.position;
                float r = diff.magnitude;
                if (r <= 0f || r > h) continue;

                Vector3 gradW = SpikyGradient(diff, h);

                if (pj.isBoundary)
                {
                    // Fluid–boundary pressure force (Akinci)
                    float termBoundary = (pi.pressure / (pi.density * pi.density));
                    pressureForce += -rho0Sq * pj.boundaryVolume * termBoundary * gradW;
                }
                else
                {
                    // Standard fluid–fluid pressure
                    float term =
                        (pi.pressure / (pi.density * pi.density)) +
                        (pj.pressure / (pj.density * pj.density));

                    pressureForce += -pj.mass * term * gradW;
                }
            }

            pi.force += pressureForce;
            pi.sphForce += pressureForce;
            masses[i] = pi;
        }
    }

    // ----------------------------------------------------------------------
    void ApplySPHViscosityForces()
    {
        float h = globalSmoothingRadius;
        int n = masses.Count;
        float visc = viscosity;

        for (int i = 0; i < n; i++)
        {
            MassPoint pi = masses[i];
            if (pi.isFixed) continue;

            Vector3 viscForce = Vector3.zero;

            for (int j = 0; j < n; j++)
            {
                if (i == j) continue;

                MassPoint pj = masses[j];
                if (pj.isFixed) continue;


                Vector3 diff = pj.position - pi.position;
                float r = diff.magnitude;
                if (r <= 0f || r > h) continue;

                float lapW = ViscosityLaplacian(r, h);

                viscForce += visc * pj.mass * (pj.velocity - pi.velocity) * (lapW / pj.density);
            }

            pi.force += viscForce;
            pi.sphForce += viscForce; // debug
        }
    }

    // ----------------------------------------------------------------------
    void Integrate()
    {
        for (int i = 0; i < masses.Count; i++)
        {
            MassPoint m = masses[i];
            if (m.isFixed) continue;

            m.velocity += (m.force / m.mass) * timeStep;

            float dampingFactor = Mathf.Clamp01(1f - dampingRatio * timeStep);
            m.velocity *= dampingFactor;

            m.position += m.velocity * timeStep;
            m.position.z = 0f;

            HandleCollisions(ref m);

            masses[i] = m;
        }
    }

    // ----------------------------------------------------------------------
    void HandleCollisions(ref MassPoint m)
    {
        Collider[] hits = Physics.OverlapSphere(m.position, collisionRadius, collisionLayerMask);

        if (hits.Length > 0)
        {
            Vector3 collisionNormal = Vector3.zero;
            float maxPenetrationDepth = float.MinValue;

            foreach (var hit in hits)
            {
                Vector3 closestPoint = hit.ClosestPoint(m.position);
                Vector3 penetrationVector = closestPoint - m.position;
                float penetrationDepth = collisionRadius - penetrationVector.magnitude;

                if (penetrationDepth > maxPenetrationDepth)
                {
                    maxPenetrationDepth = penetrationDepth;
                    collisionNormal = (m.position - closestPoint).normalized;
                }
            }

            if (maxPenetrationDepth > 0f && collisionNormal.sqrMagnitude > 0f)
            {
                m.position += collisionNormal * maxPenetrationDepth;

                float velAlongNormal = Vector3.Dot(m.velocity, collisionNormal);
                if (velAlongNormal < 0f)
                {
                    Vector3 vNormal = velAlongNormal * collisionNormal;
                    Vector3 vTangent = m.velocity - vNormal;

                    Vector3 vNormalPost = -restitution * vNormal;
                    Vector3 vTangentPost = vTangent * (1f - friction);

                    m.velocity = vNormalPost + vTangentPost;
                }
            }
        }
    }

    // ----------------------------------------------------------------------
    void OnDrawGizmos()
    {
        if (!showDebug || masses == null) return;

        // Springs
        if (springs != null)
        {
            Gizmos.color = Color.yellow;
            foreach (var s in springs)
            {
                if (s.startMassIndex < 0 || s.startMassIndex >= masses.Count ||
                    s.endMassIndex < 0 || s.endMassIndex >= masses.Count)
                    continue;

                Gizmos.DrawLine(
                    masses[s.startMassIndex].position,
                    masses[s.endMassIndex].position
                );
            }
        }

        // Particles
        foreach (var p in masses)
        {
            Color c;

            if (debugColorByDensity)
            {
                float t = Mathf.InverseLerp(debugMinDensity, debugMaxDensity, p.density);
                c = Color.Lerp(Color.blue, Color.red, t);
            }
            else
            {
                if (p.isBoundary) c = Color.black;
                else if (p.isFixed) c = Color.red;
                else c = Color.green;
            }

            Gizmos.color = c;
            Gizmos.DrawSphere(p.position, 0.05f);
        }

        // Smoothing radius
        if (debugDrawSmoothingRadius)
        {
            Gizmos.color = new Color(0f, 1f, 1f, 0.2f);
            foreach (var p in masses)
            {
                if (p.smoothingRadius > 0f)
                    Gizmos.DrawWireSphere(p.position, p.smoothingRadius);
            }
        }

        // SPH forces
        if (debugDrawSPHForces)
        {
            Gizmos.color = Color.magenta;
            foreach (var p in masses)
            {
                Vector3 start = p.position;
                Vector3 end = p.position + p.sphForce * debugForceScale;
                Gizmos.DrawLine(start, end);
            }
        }

        // Density heatmap
        if (debugDrawBackground && masses.Count > 0)
        {
            // Bounds from ALL particles (fluid + boundaries)
            float minX = float.MaxValue;
            float maxX = float.MinValue;
            float minY = float.MaxValue;
            float maxY = float.MinValue;

            foreach (var m in masses)
            {
                if (m.position.x < minX) minX = m.position.x;
                if (m.position.x > maxX) maxX = m.position.x;

                if (m.position.y < minY) minY = m.position.y;
                if (m.position.y > maxY) maxY = m.position.y;
            }

            float pad = globalSmoothingRadius;
            minX -= pad; maxX += pad;
            minY -= pad; maxY += pad;

            int resX = backgroundResolution;
            int resY = backgroundResolution;

            float dx = (maxX - minX) / resX;
            float dy = (maxY - minY) / resY;

            for (int iy = 0; iy < resY; iy++)
            {
                for (int ix = 0; ix < resX; ix++)
                {
                    Vector3 sample = new Vector3(minX + ix * dx, minY + iy * dy, 0);
                    float density = SampleDensity(sample);

                    float t = Mathf.InverseLerp(debugMinDensity, debugMaxDensity, density);

                    Color lowMid = Color.Lerp(backgroundLowColor, backgroundMidColor, t);
                    Color full = Color.Lerp(lowMid, backgroundHighColor, Mathf.SmoothStep(0f, 1f, t));

                    Gizmos.color = full;
                    Gizmos.DrawCube(sample, new Vector3(dx, dy, 0.01f));
                }
            }
        }
    }

    float SampleDensity(Vector3 p)
    {
            float density = 0f;
            float h = globalSmoothingRadius;

            foreach (var m in masses)
            {
                float r = (p - m.position).magnitude;
                float w = Poly6Kernel(r, h);

                if (!m.isBoundary)
                {
                    density += m.mass * w;
                }
                else
                {
                    density += restDensity * m.boundaryVolume * w;
                }
            }

            return density;
    }

}
