using System;
using UnityEditor.VersionControl;
using UnityEngine;

public class SpringFluid3D : MatrixAssemblyBase
{
    public override int dimension => 3;  // 3D system

    public override void BuildSystemMatrices()
    {
        int m = springs.Count;
        int n = masses.Count * 3; // x, y, z per node

        A = new float[m, n];
        C = new float[m, m];

        for (int i = 0; i < m; i++)
        {
            var spring = springs[i];
            Vector3 start = masses[spring.startMassIndex].position;
            Vector3 end = masses[spring.endMassIndex].position;
            Vector3 dir = (end - start).normalized;

            float cx = dir.x;
            float cy = dir.y;
            float cz = dir.z;

            int startIdx = spring.startMassIndex * 3;
            int endIdx = spring.endMassIndex * 3;

            // X
            A[i, startIdx] = -cx;
            A[i, endIdx] = cx;
            // Y
            A[i, startIdx + 1] = -cy;
            A[i, endIdx + 1] = cy;
            // Z
            A[i, startIdx + 2] = -cz;
            A[i, endIdx + 2] = cz;

            C[i, i] = spring.stiffness;
        }

        ComputeGlobalStiffness();
    }
}
