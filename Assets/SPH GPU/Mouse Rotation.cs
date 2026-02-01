using UnityEngine;

[RequireComponent(typeof(Camera))]
public class MouseRotation : MonoBehaviour
{
    [Header("Mouse Settings")]
    [SerializeField] float mouseSensitivity = 3.5f;

    [Header("Vertical Rotation")]
    [SerializeField] float minPitch = -85f;
    [SerializeField] float maxPitch = 85f;

    [Header("Horizontal Rotation")]
    [SerializeField] bool clampYaw = false;
    [SerializeField] float minYaw = -90f;
    [SerializeField] float maxYaw = 90f;

    [Header("Cursor")]
    [SerializeField] bool lockCursorOnStart = true;
    [SerializeField] KeyCode toggleCursorKey = KeyCode.Escape;

    [Header("Scroll Zoom (Field of View)")]
    [SerializeField] bool enableScrollZoom = true;
    [SerializeField] float zoomSpeed = 40f;
    [SerializeField] float minFov = 20f;
    [SerializeField] float maxFov = 90f;

    float yaw;
    float pitch;
    Camera cam;

    void Awake()
    {
        cam = GetComponent<Camera>();
    }

    void Start()
    {
        Vector3 angles = transform.localEulerAngles;

        yaw = NormalizeAngle(angles.y);
        pitch = NormalizeAngle(angles.x);

        cam.fieldOfView = Mathf.Clamp(cam.fieldOfView, minFov, maxFov);
        if (lockCursorOnStart) LockCursor(true);
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleCursorKey)) ToggleCursor();


        if (enableScrollZoom)
        {
            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (Mathf.Abs(scroll) > 0.0001f)
            {
                cam.fieldOfView -= scroll * zoomSpeed;
                cam.fieldOfView = Mathf.Clamp(cam.fieldOfView, minFov, maxFov);
            }
        }

        if (Cursor.lockState != CursorLockMode.Locked) return;

        float mouseX = Input.GetAxis("Mouse X");
        float mouseY = Input.GetAxis("Mouse Y");
        yaw += mouseX * mouseSensitivity * 100f * Time.deltaTime;
        pitch -= mouseY * mouseSensitivity * 100f * Time.deltaTime;

        pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
        if (clampYaw) yaw = Mathf.Clamp(yaw, minYaw, maxYaw);
        transform.localRotation = Quaternion.Euler(pitch, yaw, 0f);
    }

    void ToggleCursor()
    {
        bool locked = Cursor.lockState == CursorLockMode.Locked;
        LockCursor(!locked);
    }

    void LockCursor(bool locked)
    {
        Cursor.lockState = locked ? CursorLockMode.Locked : CursorLockMode.None;
        Cursor.visible = !locked;
    }

    static float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        return angle;
    }
}
