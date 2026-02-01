using UnityEngine;

public static class FluidLibrary
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

    // Defaults (immutable)
    static readonly FluidSettings[] defaults =
    {
        new FluidSettings { // Water
            restDensity = 1000f, gasConstant = 900f, viscosity = 1.0f,
            baseColor = new Color(0.15f, 0.45f, 0.95f, 1f)
        },
        new FluidSettings { // Oil
            restDensity = 700f, gasConstant = 700f, viscosity = 3.5f,
            baseColor = new Color(0.85f, 0.7f, 0.25f, 1f)
        },
        new FluidSettings { // Honey
            restDensity = 1900f, gasConstant = 1200f, viscosity = 40f,
            baseColor = new Color(0.9f, 0.55f, 0.12f, 1f)
        },
        new FluidSettings { // Custom default
            restDensity = 1000f, gasConstant = 900f, viscosity = 1.0f,
            baseColor = new Color(0.5f, 0.5f, 0.5f, 1f)
        }
    };

    public static int FluidCount => defaults.Length;

    const string CustomKey = "FluidLibrary.CustomSettings.v1";

    static bool customOverrideEnabled;
    static FluidSettings customOverride;

    // Small serializable helper for PlayerPrefs (JsonUtility-friendly)
    [System.Serializable]
    struct FluidSettingsSave
    {
        public float restDensity, gasConstant, viscosity;
        public float r, g, b, a;
        public bool enabled;
    }

    static FluidLibrary()
    {
        // Initialize from defaults, then attempt load
        customOverrideEnabled = false;
        customOverride = defaults[(int)FluidType.Custom];
        LoadCustomFromPrefs();
    }

    public static FluidSettings Get(FluidType type)
    {
        if (type == FluidType.Custom && customOverrideEnabled)
            return customOverride;

        return defaults[(int)type];
    }

    public static FluidSettings GetDefaultCustom() => defaults[(int)FluidType.Custom];

    public static void SetCustom(FluidSettings settings, bool save = true)
    {
        customOverrideEnabled = true;
        customOverride = settings;

        if (save)
            SaveCustomToPrefs();
    }

    public static void ResetCustomToDefault(bool save = true)
    {
        customOverrideEnabled = false;
        customOverride = defaults[(int)FluidType.Custom];

        if (save)
            SaveCustomToPrefs();
    }

    public static void SaveCustomToPrefs()
    {
        var s = new FluidSettingsSave
        {
            enabled = customOverrideEnabled,
            restDensity = customOverride.restDensity,
            gasConstant = customOverride.gasConstant,
            viscosity = customOverride.viscosity,
            r = customOverride.baseColor.r,
            g = customOverride.baseColor.g,
            b = customOverride.baseColor.b,
            a = customOverride.baseColor.a
        };

        PlayerPrefs.SetString(CustomKey, JsonUtility.ToJson(s));
        PlayerPrefs.Save();
    }

    public static void LoadCustomFromPrefs()
    {
        if (!PlayerPrefs.HasKey(CustomKey))
            return;

        string json = PlayerPrefs.GetString(CustomKey, "");
        if (string.IsNullOrEmpty(json))
            return;

        try
        {
            var s = JsonUtility.FromJson<FluidSettingsSave>(json);

            customOverride = defaults[(int)FluidType.Custom];
            customOverride.restDensity = s.restDensity;
            customOverride.gasConstant = s.gasConstant;
            customOverride.viscosity = s.viscosity;
            customOverride.baseColor = new Color(s.r, s.g, s.b, s.a);

            customOverrideEnabled = s.enabled;
        }
        catch
        {
            // If parsing fails, ignore and keep defaults.
            customOverrideEnabled = false;
            customOverride = defaults[(int)FluidType.Custom];
        }
    }

    public static void ClearSavedCustom()
    {
        PlayerPrefs.DeleteKey(CustomKey);
        PlayerPrefs.Save();
        customOverrideEnabled = false;
        customOverride = defaults[(int)FluidType.Custom];
    }
}