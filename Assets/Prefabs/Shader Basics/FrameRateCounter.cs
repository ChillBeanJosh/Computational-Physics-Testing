using UnityEngine;
using TMPro;

public class FrameRateCounter : MonoBehaviour
{
    [SerializeField] TextMeshProUGUI display;
    public enum DisplayMode { FPS, MS }
    [SerializeField] DisplayMode displayMode = DisplayMode.FPS;

    [SerializeField, Range(0.1f, 2f)] float sampleDuration = 1f; 
    int frames;
    float duration, bestDuration = float.MaxValue, worstDuration;

    private void Update()
    {
        float frameDuration = Time.unscaledDeltaTime;
        frames += 1;
        duration += frameDuration;

        if (frameDuration < bestDuration)
        {
            bestDuration = frameDuration;
        }

        if (frameDuration > worstDuration)
        {
            worstDuration = frameDuration;
        }

        if (duration >= sampleDuration)
        {
            if (displayMode == DisplayMode.FPS)
            {
                display.SetText
                (
                "FPS\n" +
                "<color=green>{0:0}</color>\n" +
                "<color=white>{1:0}</color>\n" +
                "<color=red>{2:0}</color>",

                1f / bestDuration,
                frames / duration,
                1f / worstDuration
                );
            }
            else
            {
                display.SetText
                (
                "MS\n" +
                "<color=green>{0:1}</color>\n" +
                "<color=white>{1:1}</color>\n" +
                "<color=red>{2:1}</color>",

                1000f * bestDuration,
                1000f * duration / frames,
                1000f * worstDuration
                );
            }
            

            frames = 0;
            duration = 0f;

            bestDuration = float.MaxValue;
            worstDuration = 0f;
        }
    }
}
