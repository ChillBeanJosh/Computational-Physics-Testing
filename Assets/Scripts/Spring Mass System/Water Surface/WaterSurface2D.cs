using System.Collections.Generic;
using UnityEngine;

public class WaterSurface2D : MonoBehaviour
{
    [Header("Grid Settings")]
    public int width = 20;
    public int height = 20;
    public float spacing = 0.5f;

    [Header("Mass-Spring Parameters")]
    public float mass = 1f;
    public float springStiffness = 100f;
    public float dampingAlpha = 0.05f; // Reduced damping for more visible waves
    public float gravity = 0f; // optional for water surface (usually 0)
    public float dt = 0.01f; // Increased timestep a bit

    // Internal data:
    private List<MassPoint> masses;
    private List<SpringElement> springs;

    private float[,] A; // incidence matrix
    private float[,] C; // spring constants diagonal
    private float[,] K; // stiffness matrix

    private int n; // number of masses
    private int m; // number of springs

    private float[] u;     // displacement y at time n
    private float[] uPrev; // displacement y at time n-1
    private float[] f;     // external forces

    // Mesh and visualization:
    private Mesh mesh;
    private Vector3[] vertices;

    // Flag for initial wave trigger
    private bool waveTriggered = false;

    void Start()
    {
        InitializeGrid();
        BuildSystemMatrices();

        u = new float[n];
        uPrev = new float[n];
        f = new float[n];

        // Initial conditions: rest position, zero displacement
        for (int i = 0; i < n; i++)
        {
            u[i] = 0f;
            uPrev[i] = 0f;
            f[i] = 0f;
        }

        BuildMesh();
    }

    void InitializeGrid()
    {
        masses = new List<MassPoint>();
        springs = new List<SpringElement>();

        // Create mass points on grid
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                MassPoint mp = new MassPoint();
                mp.position = new Vector3(i * spacing, 0, j * spacing);
                mp.mass = mass;
                mp.isFixed = false;

                // Fix edges to simulate boundary
                if (i == 0 || i == width - 1 || j == 0 || j == height - 1)
                    mp.isFixed = true;

                masses.Add(mp);
            }
        }

        // Create springs - connect to right and down neighbors (structural springs)
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                int current = j * width + i;

                // Connect to right neighbor
                if (i < width - 1)
                {
                    SpringElement s = new SpringElement();
                    s.startMassIndex = current;
                    s.endMassIndex = current + 1;
                    s.stiffness = springStiffness;
                    springs.Add(s);
                }

                // Connect to down neighbor
                if (j < height - 1)
                {
                    SpringElement s = new SpringElement();
                    s.startMassIndex = current;
                    s.endMassIndex = current + width;
                    s.stiffness = springStiffness;
                    springs.Add(s);
                }

                // Optional: shear springs (diagonal)
                if (i < width - 1 && j < height - 1)
                {
                    SpringElement s1 = new SpringElement();
                    s1.startMassIndex = current;
                    s1.endMassIndex = current + width + 1;
                    s1.stiffness = springStiffness * 0.7f;
                    springs.Add(s1);

                    SpringElement s2 = new SpringElement();
                    s2.startMassIndex = current + 1;
                    s2.endMassIndex = current + width;
                    s2.stiffness = springStiffness * 0.7f;
                    springs.Add(s2);
                }
            }
        }

        n = masses.Count;
        m = springs.Count;
    }

    void BuildSystemMatrices()
    {
        A = new float[m, n];
        C = new float[m, m];

        for (int i = 0; i < m; i++)
        {
            int start = springs[i].startMassIndex;
            int end = springs[i].endMassIndex;

            A[i, start] = -1f;
            A[i, end] = 1f;

            C[i, i] = springs[i].stiffness;
        }

        K = MatrixMultiply(MatrixTranspose(A), MatrixMultiply(C, A));
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space) && !waveTriggered)
        {
            int centerIndex = (height / 2) * width + (width / 2);
            u[centerIndex] = 0.5f;
            uPrev[centerIndex] = 0.5f;
            waveTriggered = true;
            Debug.Log($"Wave triggered at index {centerIndex}");
        }

        Step(dt);

        int center = (height / 2) * width + (width / 2);
        if (waveTriggered && Mathf.Abs(u[center]) < 0.01f)
        {
            waveTriggered = false;
            Debug.Log("Wave finished, ready to trigger again");
        }

        UpdateMeshVertices();
    }

    void Step(float stepDt)
    {
        // external forces (gravity usually zero for water surface)
        for (int i = 0; i < n; i++)
        {
            if (!masses[i].isFixed)
                f[i] = masses[i].mass * gravity;
            else
                f[i] = 0f;
        }

        float[] uNext = new float[n];

        for (int i = 0; i < n; i++)
        {
            if (masses[i].isFixed)
            {
                uNext[i] = 0f; // fixed position
                continue;
            }

            // compute (K*u)_i
            float Ku_i = 0f;
            for (int j = 0; j < n; j++)
                Ku_i += K[i, j] * u[j];

            // simple damping approx (can be improved)
            float dampingForce = dampingAlpha * masses[i].mass * ((u[i] - uPrev[i]) / stepDt);

            float numer = f[i] * (stepDt * stepDt) - dampingForce * stepDt - Ku_i * (stepDt * stepDt);
            uNext[i] = numer / masses[i].mass + 2f * u[i] - uPrev[i];
        }

        for (int i = 0; i < n; i++)
        {
            uPrev[i] = u[i];
            u[i] = uNext[i];
        }
    }

    void BuildMesh()
    {
        mesh = new Mesh();
        vertices = new Vector3[n];
        int[] triangles = new int[(width - 1) * (height - 1) * 6];

        for (int i = 0; i < n; i++)
            vertices[i] = masses[i].position;

        int t = 0;
        for (int j = 0; j < height - 1; j++)
        {
            for (int i = 0; i < width - 1; i++)
            {
                int current = j * width + i;
                int nextRow = current + width;

                // first triangle
                triangles[t++] = current;
                triangles[t++] = nextRow;
                triangles[t++] = current + 1;

                // second triangle
                triangles[t++] = current + 1;
                triangles[t++] = nextRow;
                triangles[t++] = nextRow + 1;
            }
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        MeshFilter mf = gameObject.GetComponent<MeshFilter>();
        if (mf == null) mf = gameObject.AddComponent<MeshFilter>();
        mf.mesh = mesh;

        MeshRenderer mr = gameObject.GetComponent<MeshRenderer>();
        if (mr == null) mr = gameObject.AddComponent<MeshRenderer>();
        // assign your material here for visualization!
    }

    void UpdateMeshVertices()
    {
        for (int i = 0; i < n; i++)
        {
            Vector3 v = masses[i].position;
            v.y = u[i];
            vertices[i] = v;
        }
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }

    // Helpers for matrix math:

    float[,] MatrixMultiply(float[,] A, float[,] B)
    {
        int aRows = A.GetLength(0);
        int aCols = A.GetLength(1);
        int bCols = B.GetLength(1);

        float[,] result = new float[aRows, bCols];

        for (int i = 0; i < aRows; i++)
            for (int j = 0; j < bCols; j++)
                for (int k = 0; k < aCols; k++)
                    result[i, j] += A[i, k] * B[k, j];

        return result;
    }

    float[,] MatrixTranspose(float[,] A)
    {
        int rows = A.GetLength(0);
        int cols = A.GetLength(1);

        float[,] result = new float[cols, rows];

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[j, i] = A[i, j];

        return result;
    }

    void OnDrawGizmos()
    {
        if (masses == null || springs == null) return;
        if (u == null || u.Length != masses.Count) return;

        // Draw Springs (Lines) with updated y displacement
        Gizmos.color = Color.yellow;
        foreach (var s in springs)
        {
            if (s.startMassIndex < masses.Count && s.endMassIndex < masses.Count)
            {
                Vector3 start = masses[s.startMassIndex].position;
                Vector3 end = masses[s.endMassIndex].position;

                // Add current vertical displacement
                start.y = u[s.startMassIndex];
                end.y = u[s.endMassIndex];

                Gizmos.DrawLine(start, end);
            }
        }

        // Draw Masses (Spheres) with updated y displacement
        for (int i = 0; i < masses.Count; i++)
        {
            Vector3 pos = masses[i].position;
            pos.y = u[i];

            Gizmos.color = masses[i].isFixed ? Color.red : Color.green;
            Gizmos.DrawSphere(pos, 0.05f);
        }
    }
}
