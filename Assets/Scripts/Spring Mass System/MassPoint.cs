using UnityEngine;

[System.Serializable]
public class MassPoint
{
    public float mass;
    public bool isFixed = false;
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 force;
}
