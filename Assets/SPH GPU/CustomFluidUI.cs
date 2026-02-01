using TMPro;
using UnityEngine;
using UnityEngine.UI;
public class CustomFluidUI : MonoBehaviour
{
    [Header("References")]
    [SerializeField] GPUSPH3D gpuSph;

    [Header("TMP Input Fields")]
    [SerializeField] TMP_InputField restDensityInput;
    [SerializeField] TMP_InputField gasConstantInput;
    [SerializeField] TMP_InputField viscosityInput;

    [Header("Optional Color Sliders (0..1)")]
    [SerializeField] Slider rSlider;
    [SerializeField] Slider gSlider;
    [SerializeField] Slider bSlider;

    [Header("Apply Settings")]
    [SerializeField] bool applyOnEndEdit = true; // apply when leaving field
    [SerializeField] bool applyOnValueChanged = false; // apply every keystroke (more spam)

    public static bool IsEditingAnyField { get; private set; }


    void Start()
    {
        if (gpuSph == null)
            Debug.LogWarning("CustomFluidUI: GPUSPH3D reference not set.");

        FluidLibrary.LoadCustomFromPrefs();
        var s = FluidLibrary.Get(FluidLibrary.FluidType.Custom);
        SetUIFromSettings(s); ;

        if (applyOnEndEdit)
        {
            restDensityInput.onEndEdit.AddListener(_ => Apply());
            gasConstantInput.onEndEdit.AddListener(_ => Apply());
            viscosityInput.onEndEdit.AddListener(_ => Apply());
        }

        if (applyOnValueChanged)
        {
            restDensityInput.onValueChanged.AddListener(_ => Apply());
            gasConstantInput.onValueChanged.AddListener(_ => Apply());
            viscosityInput.onValueChanged.AddListener(_ => Apply());
        }

        if (rSlider) rSlider.onValueChanged.AddListener(_ => Apply());
        if (gSlider) gSlider.onValueChanged.AddListener(_ => Apply());
        if (bSlider) bSlider.onValueChanged.AddListener(_ => Apply());

        if (gpuSph != null)
            gpuSph.RefreshFluidTableOnGPU();
    }

    void Update()
    {
        IsEditingAnyField =
            (restDensityInput != null && restDensityInput.isFocused) ||
            (gasConstantInput != null && gasConstantInput.isFocused) ||
            (viscosityInput != null && viscosityInput.isFocused);
    }


    public void Apply()
    {
        var s = FluidLibrary.Get(FluidLibrary.FluidType.Custom);

        s.restDensity = ReadFloat(restDensityInput, s.restDensity, 1f, 100000f);
        s.gasConstant = ReadFloat(gasConstantInput, s.gasConstant, 0f, 100000f);
        s.viscosity = ReadFloat(viscosityInput, s.viscosity, 0f, 100000f);

        if (rSlider && gSlider && bSlider)
        {
            s.baseColor = new Color(rSlider.value, gSlider.value, bSlider.value, 1f);
        }

        FluidLibrary.SetCustom(s);

        if (gpuSph != null)
            gpuSph.RefreshFluidTableOnGPU();
    }

    public void ResetToDefault()
    {
        FluidLibrary.ResetCustomToDefault();
        var s = FluidLibrary.Get(FluidLibrary.FluidType.Custom);
        SetUIFromSettings(s);

        if (gpuSph != null)
            gpuSph.RefreshFluidTableOnGPU();
    }

    void SetUIFromSettings(FluidLibrary.FluidSettings s)
    {
        if (restDensityInput) restDensityInput.text = s.restDensity.ToString("0.###");
        if (gasConstantInput) gasConstantInput.text = s.gasConstant.ToString("0.###");
        if (viscosityInput) viscosityInput.text = s.viscosity.ToString("0.###");

        if (rSlider) rSlider.value = s.baseColor.r;
        if (gSlider) gSlider.value = s.baseColor.g;
        if (bSlider) bSlider.value = s.baseColor.b;
    }

    static float ReadFloat(TMP_InputField field, float fallback, float min, float max)
    {
        if (field == null) return fallback;

        if (!float.TryParse(field.text, out float v))
            return fallback;

        return Mathf.Clamp(v, min, max);
    }
}
