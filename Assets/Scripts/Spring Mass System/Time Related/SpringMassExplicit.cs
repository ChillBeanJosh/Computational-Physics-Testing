using UnityEngine;
using System.Collections.Generic;

public class SpringMassExplicit : MonoBehaviour
{
    public SpringMassSystem baseSystem; // Reference to the system defining masses, springs, and stiffness matrix
    public float dampingAlpha = 0.0f; // Viscous damping coefficient (proportional to velocity)
    public float dt = 0.005f; // Time step size for explicit integration
    public int substeps = 1; // Number of substeps per frame (unused, could be used for stability)

    private int n; // Number of masses
    private float[] M; // Masses of each node
    private float[,] K; // Stiffness matrix from the base system
    private bool[] isFixed; // Flags indicating fixed nodes (displacement constrained)
    private float[] u;     // Current displacement vector (u^n)
    private float[] uPrev; // Previous displacement vector (u^{n-1})
    private float[] f;     // External forces vector at current step (f^n)

    void Start()
    {
        if (baseSystem == null)
        {
            Debug.LogError("SpringMassExplicit: BaseSystem not assigned!");
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
        // Initialize simulation arrays from the base spring-mass system
        K = baseSystem.K;
        n = baseSystem.masses.Count;
        M = new float[n];
        isFixed = new bool[n];

        for (int i = 0; i < n; i++)
        {
            M[i] = baseSystem.masses[i].mass;
            isFixed[i] = baseSystem.masses[i].isFixed;
        }

        // Initialize displacement and previous displacement arrays from current mass positions
        u = new float[n];
        uPrev = new float[n];

        for (int i = 0; i < n; i++)
        {
            u[i] = baseSystem.masses[i].position.y;
            uPrev[i] = u[i];
        }
    }

    void Update()
    {
        // Adaptively subdivide the timestep to maintain numerical stability if frame delta is large
        float frameDt = Time.deltaTime;
        float stepDt = dt;
        int steps = Mathf.Max(1, Mathf.CeilToInt(frameDt / stepDt));
        stepDt = frameDt / steps;

        for (int s = 0; s < steps; s++)
            Step(stepDt);

        // Update the base system's mass positions and velocities from the new displacements
        for (int i = 0; i < n; i++)
        {
            baseSystem.masses[i].position.y = u[i];
            baseSystem.masses[i].velocity.y = (u[i] - uPrev[i]) / dt; // finite difference velocity estimate
        }
    }

    /// <summary>
    /// Performs one explicit integration step using the central difference method.
    /// This is an explicit time integration scheme used in computational physics to solve
    /// second-order ODEs like M * d2u/dt2 + C * du/dt + K * u = f.
    ///
    /// Here:
    /// - M is the mass matrix (diagonal, scalar masses)
    /// - K is the stiffness matrix (from springs)
    /// - C is the damping force proportional to velocity (modeled by dampingAlpha)
    /// - u is displacement, f is external force (gravity)
    ///
    /// The scheme computes the next displacement u^{n+1} based on current and previous displacements,
    /// forces, damping, and stiffness forces.
    /// </summary>
    void Step(float stepDt)
    {
        // Initialize external forces vector f with gravity applied to free masses
        f = new float[n];
        for (int i = 0; i < n; i++)
        {
            if (!isFixed[i]) f[i] = M[i] * baseSystem.gravity; // force = m * g
            else f[i] = 0f;
        }

        float[] uNext = new float[n]; // To hold computed displacements for the next timestep

        for (int i = 0; i < n; i++)
        {
            if (isFixed[i])
            {
                // Fixed nodes remain at zero displacement (boundary condition)
                uNext[i] = u[i];
                continue;
            }

            // Compute internal elastic forces: K * u
            float Ku_i = 0f;
            for (int j = 0; j < n; j++)
                Ku_i += K[i, j] * u[j];

            // Damping force proportional to velocity (approximated as finite difference)
            // dampingForce = alpha * m * (u^n - u^{n-1}) / dt
            float dampingForce = dampingAlpha * M[i] * ((u[i] - uPrev[i]) / stepDt);

            /*
            Explicit central difference integration formula for second-order ODE:

            M * (u^{n+1} - 2u^{n} + u^{n-1}) / dt^2 + C * (u^{n} - u^{n-1}) / dt + K * u^{n} = f^{n}

            Rearranged to solve for u^{n+1}:

            u^{n+1} = (dt^2 / M) * (f^{n} - C * (u^{n} - u^{n-1}) / dt - K * u^{n}) + 2u^{n} - u^{n-1}

            where:
            - M: mass
            - C: damping coefficient
            - K: stiffness matrix
            - f: external force (gravity here)
            */
            float numer = f[i] * (stepDt * stepDt) - dampingForce * stepDt - Ku_i * (stepDt * stepDt);
            uNext[i] = numer / M[i] + 2f * u[i] - uPrev[i];
        }

        // Update displacement history for next step
        for (int i = 0; i < n; i++)
        {
            uPrev[i] = u[i];
            u[i] = uNext[i];
        }
    }
}