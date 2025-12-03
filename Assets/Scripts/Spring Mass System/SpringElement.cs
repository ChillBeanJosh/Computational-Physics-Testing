using UnityEngine;

[System.Serializable]
public class SpringElement
{
    [Header("Mass Indices")]
    public int startMassIndex;
    public int endMassIndex;

    [Header("Spring Properties")]
    public float stiffness = 10f;
    public float damping = 2f;

    [HideInInspector]
    public float restLength;
}
