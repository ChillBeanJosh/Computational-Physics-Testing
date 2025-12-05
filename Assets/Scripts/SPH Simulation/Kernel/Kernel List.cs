using UnityEngine;

public class KernelList : MonoBehaviour
{
    //----------------------------------------------------------------------------------------------------------------------------
    //2D Poly6 Kernel (Density):
    // W(r,h) = 4 / (π h^8) * (h^2 - r^2)^3
    public float Poly6(float r, float h)
    {
        if (r < 0f || r > h) return 0f;

        float h2 = h * h;
        float r2 = r * r;
        float term = h2 - r2;

        float constant = 4f / (Mathf.PI * Mathf.Pow(h, 8));

        return constant * term * term * term;
    }

    //----------------------------------------------------------------------------------------------------------------------------
    //2D Spiky Gradient (Pressure):
    // ∇W = -30 / (π h^5) * (h - r)^2 * r̂
    public Vector3 SpikyGradient(Vector3 dir, float r, float h)
    {
        if (r <= 0f || r >= h) return Vector3.zero;

        float constant = -30f / (Mathf.PI * Mathf.Pow(h, 5));
        float term = (h - r) * (h - r);

        return constant * term * (dir / r);
    }

    //----------------------------------------------------------------------------------------------------------------------------
    //2D Viscosity Laplacian:
    // ∇²W = 40 / (π h^5) * (h - r)
    public float ViscosityLaplacian(float r, float h)
    {
        if (r < 0f || r > h) return 0f;

        float constant = 40f / (Mathf.PI * Mathf.Pow(h, 5));
        return constant * (h - r);
    }
}
