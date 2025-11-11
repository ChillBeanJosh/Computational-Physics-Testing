using System.Collections.Generic;
using UnityEngine;

public static class MatrixUtils
{
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Element-wise addition of two vectors
    public static float[] Add(float[] a, float[] b)
    {
        if (a.Length != b.Length)
            throw new System.ArgumentException("Vectors must be the same length");

        float[] result = new float[a.Length];
        for (int i = 0; i < a.Length; i++)
        {
            result[i] = a[i] + b[i];
        }
        return result;
    }

    // Element-wise subtraction of two vectors
    public static float[] Subtract(float[] a, float[] b)
    {
        if (a.Length != b.Length)
            throw new System.ArgumentException("Vectors must be the same length");

        float[] result = new float[a.Length];
        for (int i = 0; i < a.Length; i++)
        {
            result[i] = a[i] - b[i];
        }
        return result;
    }
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //Matrix Multiplication of a 2D Matrix (RowA * ColB)
    public static float[,] Multiply(float[,] A, float[,] B)
    {
        //Matrix A Parameters:
        int aRows = A.GetLength(0);
        int aCols = A.GetLength(1);

        //Matrix B Parameters:
        int bCols = B.GetLength(1);

        //Intitial Size Set Of Empty Matrix For Result:
        float[,] result = new float[aRows, bCols];

        //Result Row:
        for (int i = 0; i < aRows; i++)
        {
            //Result Col:
            for (int j = 0; j < bCols; j++)
            {
                //Sum Index:
                for (int k = 0; k < aCols; k++)
                {
                    result[i, j] += A[i, k] * B[k, j];
                }
            }
        }

        //Return The Newly Calculated Matrix:
        return result;
    }

    // Multiply 2D Matrix with Vector:
    public static float[] Multiply(float[,] matrix, float[] vector)
    {
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);

        if (cols != vector.Length)
            throw new System.ArgumentException("Matrix columns must match vector length");

        float[] result = new float[rows];
        for (int i = 0; i < rows; i++)
        {
            float sum = 0f;
            for (int j = 0; j < cols; j++)
            {
                sum += matrix[i, j] * vector[j];
            }
            result[i] = sum;
        }
        return result;
    }

    // Element-wise multiplication of vector by scalar
    public static float[] Multiply(float[] vector, float scalar)
    {
        float[] result = new float[vector.Length];
        for (int i = 0; i < vector.Length; i++)
        {
            result[i] = vector[i] * scalar;
        }
        return result;
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //Matrix Transpose Swaping Row & Col:
    public static float[,] Transpose(float[,] A)
    {
        //Matrix A Parameters:
        int rows = A.GetLength(0);
        int cols = A.GetLength(1);

        //New Matrix Being the Row & Col Swapped:
        float[,] result = new float[cols, rows];

        //Reflect Across The Diagonal:
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result[j, i] = A[i, j];
            }
        }

        //Return The Newly Calculated Matrix:
        return result;
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Solve K * u = f with fixed constraints (displacement zero)
    // Uses naive Gauss-Seidel-like iteration for small systems
    public static float[] SolveSystem(float[,] K, float[] f, List<MassPoint> masses)
    {
        int totalDOF = f.Length;
        int dofPerNode = Mathf.FloorToInt((float)totalDOF / masses.Count);

        // Clone to avoid modifying original
        float[,] Kmod = (float[,])K.Clone();
        float[] fmod = (float[])f.Clone();

        // Apply boundary (fixed) conditions
        for (int i = 0; i < masses.Count; i++)
        {
            if (masses[i].isFixed)
            {
                for (int d = 0; d < dofPerNode; d++)
                {
                    int idx = i * dofPerNode + d;
                    for (int j = 0; j < totalDOF; j++)
                    {
                        Kmod[idx, j] = 0f;
                        Kmod[j, idx] = 0f;
                    }
                    Kmod[idx, idx] = 1f;
                    fmod[idx] = 0f;
                }
            }
        }

        return GaussianElimination(Kmod, fmod);
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Gaussian elimination solver for A*x=b
    public static float[] GaussianElimination(float[,] A, float[] b)
    {
        int n = b.Length;
        float[,] mat = (float[,])A.Clone();
        float[] vec = (float[])b.Clone();

        // Forward elimination
        for (int k = 0; k < n; k++)
        {
            // Find pivot
            int maxRow = k;
            float maxVal = Mathf.Abs(mat[k, k]);
            for (int i = k + 1; i < n; i++)
            {
                if (Mathf.Abs(mat[i, k]) > maxVal)
                {
                    maxVal = Mathf.Abs(mat[i, k]);
                    maxRow = i;
                }
            }

            // Swap rows if needed
            if (maxRow != k)
            {
                for (int j = 0; j < n; j++)
                {
                    float temp = mat[k, j];
                    mat[k, j] = mat[maxRow, j];
                    mat[maxRow, j] = temp;
                }
                float tmpB = vec[k];
                vec[k] = vec[maxRow];
                vec[maxRow] = tmpB;
            }

            // Eliminate
            for (int i = k + 1; i < n; i++)
            {
                float c = mat[i, k] / mat[k, k];
                for (int j = k; j < n; j++)
                    mat[i, j] -= c * mat[k, j];
                vec[i] -= c * vec[k];
            }
        }

        // Back substitution
        float[] x = new float[n];
        for (int i = n - 1; i >= 0; i--)
        {
            float sum = vec[i];
            for (int j = i + 1; j < n; j++)
                sum -= mat[i, j] * x[j];
            x[i] = sum / mat[i, i];
        }
        return x;
    }
    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Create diagonal matrix
    public static float[,] CreateDiagonalMatrix(float[] diag)
    {
        int n = diag.Length;
        float[,] result = new float[n, n];
        for (int i = 0; i < n; i++) result[i, i] = diag[i];
        return result;
    }
}