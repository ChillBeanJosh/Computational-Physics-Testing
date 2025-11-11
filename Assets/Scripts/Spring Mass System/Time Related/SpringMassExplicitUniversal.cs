using UnityEngine;

public class SpringMassExplicitUniversal : MonoBehaviour
{
    public MatrixAssemblyBase baseSystem; // base system with K, masses, etc.
    public float dampingAlpha = 0.0f;     // damping coefficient
    public float dt = 0.005f;             // timestep
    public int substeps = 1;

    private int n;          // number of nodes
    private int dim;        // degrees of freedom per node (1, 2, or 3)
    private float[] M;      // masses, length = n * dim (usually repeated scalar masses per DOF)
    private float[,] K;     // stiffness matrix (n*dim x n*dim)
    private bool[] isFixed; // fixed DOFs flags, length = n * dim

    private float[] u;      // current displacement vector (length n*dim)
    private float[] uPrev;  // previous displacement vector
    private float[] f;      // external forces vector

    void Start()
    {
        if (baseSystem == null)
        {
            Debug.LogError("SpringMassExplicitUniversal: BaseSystem not assigned!");
            enabled = false;
            return;
        }

        if (baseSystem.K == null)
        {
            Debug.LogWarning("BaseSystem.K not built yet — rebuilding now.");
            baseSystem.BuildSystemMatrices();
        }

        InitializeFromBase();
    }

    void InitializeFromBase()
    {
        n = baseSystem.masses.Count;
        dim = baseSystem.dimension;

        int totalDOFs = n * dim;

        // Initialize arrays
        M = new float[totalDOFs];
        isFixed = new bool[totalDOFs];
        u = new float[totalDOFs];
        uPrev = new float[totalDOFs];

        K = baseSystem.K; // expect baseSystem.K to be (n*dim) x (n*dim)

        // Fill M and isFixed - assuming baseSystem.masses have scalar mass and isFixed per node
        // For DOFs, replicate mass per DOF and replicate fixed flags per DOF
        for (int i = 0; i < n; i++)
        {
            float mass = baseSystem.masses[i].mass;
            bool fixedFlag = baseSystem.masses[i].isFixed;

            for (int d = 0; d < dim; d++)
            {
                int idx = i * dim + d;
                M[idx] = mass;
                isFixed[idx] = fixedFlag;

                // Initialize displacement from mass position component based on dimension
                switch (dim)
                {
                    case 1:
                        u[idx] = baseSystem.masses[i].position.y; // assuming 1D along y
                        break;
                    case 2:
                        if (d == 0) u[idx] = baseSystem.masses[i].position.x;
                        else if (d == 1) u[idx] = baseSystem.masses[i].position.y;
                        break;
                    case 3:
                        if (d == 0) u[idx] = baseSystem.masses[i].position.x;
                        else if (d == 1) u[idx] = baseSystem.masses[i].position.y;
                        else if (d == 2) u[idx] = baseSystem.masses[i].position.z;
                        break;
                }

                uPrev[idx] = u[idx];
            }
        }
    }

    void Update()
    {
        float frameDt = Time.deltaTime;
        float stepDt = dt;
        int steps = Mathf.Max(1, Mathf.CeilToInt(frameDt / stepDt));
        stepDt = frameDt / steps;

        for (int s = 0; s < steps; s++)
            Step(stepDt);

        // Write back to baseSystem masses positions and estimate velocities
        for (int i = 0; i < n; i++)
        {
            switch (dim)
            {
                case 1:
                    baseSystem.masses[i].position.y = u[i * dim];
                    baseSystem.masses[i].velocity.y = (u[i * dim] - uPrev[i * dim]) / dt;
                    break;

                case 2:
                    baseSystem.masses[i].position.x = u[i * dim];
                    baseSystem.masses[i].position.y = u[i * dim + 1];
                    baseSystem.masses[i].velocity.x = (u[i * dim] - uPrev[i * dim]) / dt;
                    baseSystem.masses[i].velocity.y = (u[i * dim + 1] - uPrev[i * dim + 1]) / dt;
                    break;

                case 3:
                    baseSystem.masses[i].position.x = u[i * dim];
                    baseSystem.masses[i].position.y = u[i * dim + 1];
                    baseSystem.masses[i].position.z = u[i * dim + 2];
                    baseSystem.masses[i].velocity.x = (u[i * dim] - uPrev[i * dim]) / dt;
                    baseSystem.masses[i].velocity.y = (u[i * dim + 1] - uPrev[i * dim + 1]) / dt;
                    baseSystem.masses[i].velocity.z = (u[i * dim + 2] - uPrev[i * dim + 2]) / dt;
                    break;
            }
        }
    }

    void Step(float stepDt)
    {
        int totalDOFs = n * dim;

        // Initialize external forces vector f (gravity) per DOF
        f = new float[totalDOFs];

        for (int i = 0; i < n; i++)
        {
            for (int d = 0; d < dim; d++)
            {
                int idx = i * dim + d;

                if (!isFixed[idx])
                {
                    // Gravity force only in y-direction (d == 1) for 2D/3D and 1D along y
                    if (dim == 1 && d == 0) f[idx] = M[idx] * baseSystem.gravity;
                    else if ((dim == 2 || dim == 3) && d == 1) f[idx] = M[idx] * baseSystem.gravity;
                    else f[idx] = 0f;
                }
                else
                {
                    f[idx] = 0f;
                }
            }
        }

        float[] uNext = new float[totalDOFs];

        for (int i = 0; i < totalDOFs; i++)
        {
            if (isFixed[i])
            {
                uNext[i] = u[i]; // fixed DOFs maintain displacement
                continue;
            }

            // Compute internal elastic forces: K * u (row i)
            float Ku_i = 0f;
            for (int j = 0; j < totalDOFs; j++)
                Ku_i += K[i, j] * u[j];

            // Damping force proportional to velocity
            float dampingForce = dampingAlpha * M[i] * ((u[i] - uPrev[i]) / stepDt);

            float numer = f[i] * (stepDt * stepDt) - dampingForce * stepDt - Ku_i * (stepDt * stepDt);
            uNext[i] = numer / M[i] + 2f * u[i] - uPrev[i];
        }

        // Update displacement history
        for (int i = 0; i < totalDOFs; i++)
        {
            uPrev[i] = u[i];
            u[i] = uNext[i];
        }
    }
}
