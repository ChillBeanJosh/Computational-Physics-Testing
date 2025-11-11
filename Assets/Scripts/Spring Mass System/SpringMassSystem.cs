using UnityEngine;
using System.Collections.Generic;

public class SpringMassSystem : MonoBehaviour
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
    private float[,] A;
    private float[,] C;
    public float[,] K;
    [Space]

    [Header("1D - Matrices: ")]
    private float[] u;  // Displacements
    private float[] f;  // Forces


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

    public void BuildSystemMatrices()
    {
        //Variables To Declare Size of Matrix:
        int m = springs.Count;
        int n = masses.Count;

        //Declaration of Empty 2D Matrix Size [ROW x COL]:
        A = new float[m, n];
        C = new float[m, m];

        //For Each Row:
        for (int i = 0; i < m; i++)
        {
            //Set the Row's Start/End Points:
            int start = springs[i].startMassIndex;
            int end = springs[i].endMassIndex;

            //
            A[i, start] = -1f;
            A[i, end] = 1f;

            //Square Matrix:
            C[i, i] = springs[i].stiffness;
        }

        //K = A^(T) * C * A
        K = MatrixUtils.Multiply(MatrixUtils.Transpose(A), MatrixUtils.Multiply(C, A));
    }

    void ApplyGravity()
    {
        //Declaration of Empty 1D Matrix Size:
        f = new float[masses.Count];

        //For Each Mass In The System:
        for (int i = 0; i < masses.Count; i++)
        {
            //If It Not Fixed:
            if (!masses[i].isFixed)
            {
                f[i] = masses[i].mass * gravity;
            }
            //If It Is Fixed:
            else
            {
                f[i] = 0f;
            }
        }
    }

    void SolveSystem()
    {
        // For fixed nodes: set displacement = 0
        // For free nodes: solve K * u = f  (small systems can use naive inversion)
        u = MatrixUtils.SolveSystem(K, f, masses);
    }

    void UpdateMassPositions()
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
