using UnityEngine;

public class TankRotator : MonoBehaviour
{
    [Header("Rotation Speed")]
    [SerializeField] float rotationSpeed = 45f;

    [Header("Keybinds")]
    [SerializeField] KeyCode rotatePosX = KeyCode.W; 
    [SerializeField] KeyCode rotateNegX = KeyCode.S;

    [SerializeField] KeyCode rotatePosY = KeyCode.A; 
    [SerializeField] KeyCode rotateNegY = KeyCode.D; 

    [SerializeField] KeyCode rotatePosZ = KeyCode.E; 
    [SerializeField] KeyCode rotateNegZ = KeyCode.Q; 

    [Header("Options")]
    [SerializeField] bool useLocalSpace = true;
    [SerializeField] bool enableResetKey = true;
    [SerializeField] KeyCode resetKey = KeyCode.R;
    Quaternion startRotation;

    void Awake()
    {
        startRotation = transform.rotation;
    }

    void Update()
    {
        Vector3 axisInput = Vector3.zero;

        if (Input.GetKey(rotatePosX)) axisInput.x += 1f;
        if (Input.GetKey(rotateNegX)) axisInput.x -= 1f;

        if (Input.GetKey(rotatePosY)) axisInput.y += 1f;
        if (Input.GetKey(rotateNegY)) axisInput.y -= 1f;

        if (Input.GetKey(rotatePosZ)) axisInput.z += 1f;
        if (Input.GetKey(rotateNegZ)) axisInput.z -= 1f;

        if (axisInput != Vector3.zero)
        {
            Vector3 eulerDelta = axisInput * rotationSpeed * Time.deltaTime;

            if (useLocalSpace)
            {
                transform.Rotate(eulerDelta, Space.Self);
            }
            else
            {
                transform.Rotate(eulerDelta, Space.World);
            }
        }

        if (enableResetKey && Input.GetKeyDown(resetKey))
        {
            transform.rotation = startRotation;
        }
    }
}
