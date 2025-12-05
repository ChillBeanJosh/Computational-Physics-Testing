using UnityEngine;

public class KernelList : MonoBehaviour
{
    public float W;


    //All:
    public float Poly6(float r, float h)
    {
        //Otherwise:
        if (r < 0 || r > h) return W = 0;

        var Function = (315f) / (64f * Mathf.PI * Mathf.Pow(h, 9));

        var term = (h * h) - (r * r);
        var coefficient = term * term * term;


        return W = coefficient;
    }

    //Pressure Kernel:
    public float Spiky(float r, float h)
    {
        //Otherwise:
        if (r < 0 || r > h) return W = 0;

        var Function = (15f) / (Mathf.PI * Mathf.Pow(h, 6));

        var term = (h - r);
        var coefficient = term * term * term;

        return W = coefficient;
    }

    //Viscosity Kernel:
    public float Viscosity(float r, float h)
    {
        //Otherwise:
        if (r < 0 || r > h) return W = 0;

        var Function = (15f) / (2 * Mathf.PI * Mathf.Pow(h, 3));

        var coefficient = -(Mathf.Pow(r, 3) / 2 * Mathf.Pow(h, 3)) + (Mathf.Pow(r, 2) / Mathf.Pow(h, 2)) + (h / 2 * r) - 1;

        return W = coefficient;
    }
}
