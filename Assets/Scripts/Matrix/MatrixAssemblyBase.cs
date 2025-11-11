using System.Collections.Generic;
using UnityEngine;

public abstract class MatrixAssemblyBase : MonoBehaviour
{
    [Header("Spring & Mass Initialization:")]
    public List<MassPoint> masses;
    public List<SpringElement> springs;
    public float gravity = -9.81f;
    [Space]

    [Header("Debugging: ")]
    public bool showDebug = true;
    [Space]

    [Header("2D - Matrices:")]
    protected float[,] A;
    protected float[,] C;
    public float[,] K;
    [Space]

    [Header("1D - Matrices: ")]
    protected float[] u;  // Displacements
    protected float[] f;  // Forces

    // Abstract property forces subclasses to specify dimension
    public abstract int dimension { get; }


    void Start()
    {
        BuildSystemMatrices();
    }

    void Update()
    {
        ApplyGravity();
        SolveSystem();
        UpdateMassPositions();
    }

    public abstract void BuildSystemMatrices();

    protected virtual void ApplyGravity()
    {
        //Declaration of Empty 1D Matrix Size:
        f = new float[masses.Count];

        //For Each Mass In The System:
        for (int i = 0; i < masses.Count; i++)
        {
            f[i] = masses[i].isFixed ? 0f : masses[i].mass * gravity;
        }
    }

    protected virtual void SolveSystem()
    {
        // For fixed nodes: set displacement = 0
        // For free nodes: solve K * u = f  (small systems can use naive inversion)
        u = MatrixUtils.SolveSystem(K, f, masses);
    }

    protected virtual void UpdateMassPositions()
    {
        //For Each Exisiting Mass:
        for (int i = 0; i < masses.Count; i++)
        {
            //If It Is Not A Fixed Mass:
            if (!masses[i].isFixed)
            {
                //Update Position and Velocity:
                masses[i].position.y += u[i] * Time.deltaTime;
                masses[i].velocity.y = u[i];
            }
        }
    }
    public float[,] ComputeGlobalStiffness()
    {
        var At = MatrixUtils.Transpose(A);
        K = MatrixUtils.Multiply(At, MatrixUtils.Multiply(C, A));
        return K;
    }

    public virtual float[] ComputeForces(float[] displacements)
    {
        // f = A^T * C * A * u
        var temp = MatrixUtils.Multiply(C, MatrixUtils.Multiply(A, displacements));
        var result = MatrixUtils.Multiply(MatrixUtils.Transpose(A), temp);
        return result;
    }

    protected void ApplyConstraintsToSystem()
    {
        int n = K.GetLength(0);

        for (int i = 0; i < masses.Count; i++)
        {
            if (masses[i].isFixed)
            {
                int idxX = i * 2;     // X DOF index
                int idxY = i * 2 + 1; // Y DOF index

                // For X DOF
                for (int col = 0; col < n; col++)
                {
                    K[idxX, col] = 0f;
                    K[col, idxX] = 0f;
                }
                K[idxX, idxX] = 1f;
                f[idxX] = 0f;

                // For Y DOF
                for (int col = 0; col < n; col++)
                {
                    K[idxY, col] = 0f;
                    K[col, idxY] = 0f;
                }
                K[idxY, idxY] = 1f;
                f[idxY] = 0f;
            }
        }
    }

    protected bool HasRigidBodyMode(float[,] K)
    {
        int n = K.GetLength(0);
        for (int i = 0; i < n; i++)
        {
            if (Mathf.Approximately(K[i, i], 0f))
            {
                Debug.LogWarning($"Rigid body mode detected: zero diagonal at index {i}");
                return true;
            }
        }
        return false;
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //SPRING VISUALIZATION:

    void OnDrawGizmos()
    {
        if (masses == null || springs == null) return;

        // Draw Springs (Lines)
        Gizmos.color = Color.yellow;
        foreach (var s in springs)
        {
            if (s.startMassIndex < masses.Count && s.endMassIndex < masses.Count)
            {
                Vector3 start = masses[s.startMassIndex].position;
                Vector3 end = masses[s.endMassIndex].position;
                Gizmos.DrawLine(start, end);
            }
        }

        // Draw Masses (Spheres)
        foreach (var m in masses)
        {
            Gizmos.color = m.isFixed ? Color.red : Color.green;
            Gizmos.DrawSphere(m.position, 0.05f);
        }
    }
}
