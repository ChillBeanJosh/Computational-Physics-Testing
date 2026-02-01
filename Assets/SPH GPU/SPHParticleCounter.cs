using TMPro;
using UnityEngine;

public class SPHParticleCounter : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Reference to the GPUSPH3D simulator")]
    [SerializeField] GPUSPH3D gpuSph;

    [Header("Display")]
    [SerializeField] string prefix = "Particles: ";
    [SerializeField] string format = "N0"; // N0 = 1,234 formatting
    [SerializeField] Color normalColor = Color.white;

    TextMeshProUGUI tmp;

    void Awake()
    {
        tmp = GetComponent<TextMeshProUGUI>();
    }

    void Update()
    {
        if (gpuSph == null)
        {
            tmp.text = prefix + "—";
            return;
        }

        int count = gpuSph.ActiveParticleCount;
        tmp.text = prefix + count.ToString(format);
        tmp.color = normalColor;
    }
}
