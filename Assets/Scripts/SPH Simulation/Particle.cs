using UnityEngine;

[System.Serializable]
public class Particle
{
    [Header("Particle Parameters: ")]
    public float Mass;
    public float RestDensity;
    public float ViscosityCoefficient;
    public float GasConstant;
    [Space]
    public float ColorAttribute;
    public float Temperature;

    [Header("Particle Test Information: ")]
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 force;

    [Header("SPH Properties")]
    public float smoothingRadius;
    public float density;
    public float pressure;
}
