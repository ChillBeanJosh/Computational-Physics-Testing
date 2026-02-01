using UnityEngine;

public class DisableAxis : MonoBehaviour
{
    [SerializeField] GameObject[] targets;

    bool isEnabled = true;
    public void Toggle()
    {
        isEnabled = !isEnabled;

        for (int i = 0; i < targets.Length; i++)
        {
            if (targets[i] != null)
                targets[i].SetActive(isEnabled);
        }
    }
    public void EnableAll()
    {
        isEnabled = true;
        SetAll(true);
    }

    public void DisableAll()
    {
        isEnabled = false;
        SetAll(false);
    }

    void SetAll(bool state)
    {
        for (int i = 0; i < targets.Length; i++)
        {
            if (targets[i] != null)
                targets[i].SetActive(state);
        }
    }
}
