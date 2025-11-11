using UnityEngine;
using System.Collections.Generic;

public class SpringMassSystem : MatrixAssemblyBase
{
    public override int dimension => 1;  // 1D system
    public override void BuildSystemMatrices()
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
        ComputeGlobalStiffness();
    }
}
