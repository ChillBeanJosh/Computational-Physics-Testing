using UnityEngine;

public class KernelList : MonoBehaviour
{
    //----------------------------------------------------------------------------------------------------------------------------
    // 3D Poly6 Kernel (Density):
    // W(r,h) = 315 / (64 π h^9) * (h^2 - r^2)^3
    public float Poly6(float r, float h)
    {
        if (r < 0f || r > h) return 0f;

        float h2 = h * h;
        float r2 = r * r;
        float term = h2 - r2;

        float constant = 315f / (64f * Mathf.PI * Mathf.Pow(h, 9));

        return constant * term * term * term;
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // 3D Spiky Gradient (Pressure):
    // ∇W = -45 / (π h^6) * (h - r)^2 * r̂
    public Vector3 SpikyGradient(Vector3 dir, float r, float h)
    {
        if (r <= 0f || r >= h) return Vector3.zero;

        float constant = -45f / (Mathf.PI * Mathf.Pow(h, 6));
        float term = (h - r) * (h - r);

        return constant * term * (dir / r);
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // 3D Viscosity Laplacian:
    // ∇²W = 45 / (π h^6) * (h - r)
    public float ViscosityLaplacian(float r, float h)
    {
        if (r < 0f || r > h) return 0f;

        float constant = 45f / (Mathf.PI * Mathf.Pow(h, 6));
        return constant * (h - r);
    }
}
