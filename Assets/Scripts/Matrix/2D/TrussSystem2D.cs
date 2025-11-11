using System;
using System.Collections.Generic;
using UnityEditor.VersionControl;
using UnityEngine;

public class TrussSystem2D : MatrixAssemblyBase
{
    [Header("Additional Forces")]
    public float horizontalForce = 0f;  // external force on x axis per mass

    public override int dimension => 2;  // 2 DOFs per node (x and y)

    public override void BuildSystemMatrices()
    {
        int numNodes = masses.Count;
        int dof = dimension * numNodes;

        A = new float[springs.Count, dof];
        C = new float[springs.Count, springs.Count];

        for (int i = 0; i < springs.Count; i++)
        {
            SpringElement s = springs[i];
            int start = s.startMassIndex;
            int end = s.endMassIndex;

            Vector3 posStart = masses[start].position;
            Vector3 posEnd = masses[end].position;

            Vector2 dir = new Vector2(posEnd.x - posStart.x, posEnd.y - posStart.y);
            float length = dir.magnitude;
            if (length > 1e-6f)
                dir /= length;
            else
                dir = Vector2.zero;

            int startX = start * dimension;
            int startY = startX + 1;
            int endX = end * dimension;
            int endY = endX + 1;

            A[i, startX] = -dir.x;
            A[i, startY] = -dir.y;
            A[i, endX] = dir.x;
            A[i, endY] = dir.y;

            C[i, i] = s.stiffness;
        }

        K = ComputeGlobalStiffness();

        f = new float[dof];
        u = new float[dof];
    }

    protected override void ApplyGravity()
    {
        int dof = dimension * masses.Count;
        f = new float[dof];

        for (int i = 0; i < masses.Count; i++)
        {
            if (masses[i].isFixed)
            {
                f[i * dimension] = 0f;
                f[i * dimension + 1] = 0f;
            }
            else
            {
                f[i * dimension] = masses[i].mass * horizontalForce;
                f[i * dimension + 1] = masses[i].mass * gravity;
            }
        }
    }

    protected override void UpdateMassPositions()
    {
        int dof = dimension * masses.Count;
        for (int i = 0; i < masses.Count; i++)
        {
            if (!masses[i].isFixed)
            {
                int idxX = i * dimension;
                int idxY = idxX + 1;

                Vector3 pos = masses[i].position;
                pos.x += u[idxX] * Time.deltaTime;
                pos.y += u[idxY] * Time.deltaTime;
                masses[i].position = pos;

                masses[i].velocity.x = u[idxX];
                masses[i].velocity.y = u[idxY];
            }
        }
    }

    protected override void SolveSystem()
    {
        // Apply constraints only if fixed, but no stability checks
        ApplyConstraintsToSystem();

        u = MatrixUtils.SolveSystem(K, f, masses);
    }
}
