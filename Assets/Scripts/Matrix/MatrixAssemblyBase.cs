using System.Collections.Generic;
using UnityEngine;

//FEM Static Solver:
//Base Class For Matrix Assembly and System Solving:
//Handles General Operations Common To All Spring-Mass Systems:
//Derive Specific Implementations For Different Dimensions From This Class:
// e.g., 1D, 2D, 3D Systems.
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
    protected float[,] A; //Global Connectivity Matrix
    protected float[,] C; //Global Constitutive Matrix
    public float[,] K; //Global Stiffness Matrix
    [Space]

    [Header("1D - Matrices: ")]
    protected float[] u; //Displacements
    protected float[] f; //Forces

    // Dimension of the system (e.g., 2 for 2D)
    public abstract int dimension { get; }


    void Start()
    {
        //Initial Build Of System Matrices:
        BuildSystemMatrices();
    }

    void Update()
    {
        //Each Frame, Apply Gravity, Solve System, Update Positions:
        ApplyGravity();
        SolveSystem();
        UpdateMassPositions();
    }

    public abstract void BuildSystemMatrices();

    protected virtual void ApplyGravity()
    {
        //Declaration of Empty 1D Matrix Size:
        //Forces Matrix:
        f = new float[masses.Count];

        //For Each Mass In The System:
        //Apply Gravity Force (If Not Fixed):
        for (int i = 0; i < masses.Count; i++)
        {
            f[i] = masses[i].isFixed ? 0f : masses[i].mass * gravity;
        }
    }

    protected virtual void SolveSystem()
    {
        //For fixed nodes: set displacement = 0 
        //For free nodes: solve K * u = f  (small systems can use naive inversion)
        //Apply Constraints To System:
        u = MatrixUtils.SolveSystem(K, f, masses);
    }

    protected virtual void UpdateMassPositions()
    {
        //For Each Exisiting Mass:
        //Update Their Positions Based On Computed Displacements:
        for (int i = 0; i < masses.Count; i++)
        {
            //If It Is Not A Fixed Mass:
            if (!masses[i].isFixed)
            {
                //Update Position and Velocity:

                //X Direction:
                //Update Position:
                masses[i].position.y += u[i] * Time.deltaTime;

                //Y Direction:
                //Update Position:
                masses[i].velocity.y = u[i];
            }
        }
    }
    public float[,] ComputeGlobalStiffness()
    {
        // K = A^T * C * A
        // Compute K
        var At = MatrixUtils.Transpose(A);
        K = MatrixUtils.Multiply(At, MatrixUtils.Multiply(C, A));
        return K;
    }

    public virtual float[] ComputeForces(float[] displacements)
    {
        //f = A^T * C * A * u
        //Compute f
        var temp = MatrixUtils.Multiply(C, MatrixUtils.Multiply(A, displacements));
        var result = MatrixUtils.Multiply(MatrixUtils.Transpose(A), temp);
        return result;
    }

    protected void ApplyConstraintsToSystem()
    {
        //Modify K and f to account for fixed masses:
        //For Each Mass In The System:
        int n = K.GetLength(0);

        //Apply Constraints:
        for (int i = 0; i < masses.Count; i++)
        {
            //If The Mass Is Fixed:
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
