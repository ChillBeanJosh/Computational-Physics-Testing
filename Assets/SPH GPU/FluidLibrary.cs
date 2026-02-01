using UnityEngine;

public class FluidLibrary : MonoBehaviour
{
    // Expandable: add more fluids here later.
    public enum FluidType { Water = 0, Oil = 1, Honey = 2, Custom = 3 }

    [System.Serializable]
    public struct FluidSettings
    {
        public float restDensity;   // rho0
        public float gasConstant;   // k
        public float viscosity;     // mu
        public Color baseColor;     // for shading
    }

    // Default values (tune as you like)
    // Note: restDensity ratios matter, but stability depends heavily on timestep/k.
    static readonly FluidSettings[] fluids =
    {
        new FluidSettings { 
            restDensity = 1000f,
            gasConstant = 900f,
            viscosity   = 1.0f,
            baseColor   = new Color(0.15f, 0.45f, 0.95f, 1f)
        },
        new FluidSettings { 
            restDensity = 700f,     
            gasConstant = 700f,     
            viscosity   = 3.5f,     
            baseColor   = new Color(0.85f, 0.7f, 0.25f, 1f)
        },
        new FluidSettings { 
            restDensity = 1900f,    
            gasConstant = 1200f,    
            viscosity   = 40f,     
            baseColor   = new Color(0.9f, 0.55f, 0.12f, 1f)
        },
        new FluidSettings { 
            restDensity = 1000f,    
            gasConstant = 900f,     
            viscosity   = 1.0f,     
            baseColor   = new Color(0.5f, 0.5f, 0.5f, 1f)
        }
    };

    public static int FluidCount => fluids.Length;

    public static FluidSettings Get(FluidType type) => fluids[(int)type];
}
