using UnityEngine;

[System.Serializable]
public class MassPoint
{
    [Header("Mass Point Properties")]
    public float mass;
    public bool isFixed = false;

    [Header("State Variables")]
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 force;

    [Header("SPH Properties")]
    public float smoothingRadius = 0.15f;
    [Space]
    public float density = 0f;            
    public float pressure;
    [Space]
    public bool isBoundary;      // true for Akinci boundary particles
    public float boundaryVolume;  // precomputed V_b

    [Header("SPH Coefficients")]
    // For debug (to draw gradient / SPH force directions)
    [HideInInspector] public Vector3 sphForce;
}
