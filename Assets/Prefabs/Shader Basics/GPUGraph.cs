using UnityEngine;

public class GPUGraph : MonoBehaviour
{
    const int maxResolution = 1000;
    [SerializeField, Range(10, maxResolution)] int resolution = 10;

    [SerializeField] FunctionLibrary.FunctionName function;
    public enum TransitionMode { Cycle, Random }
    [SerializeField] TransitionMode transitionMode;
    [SerializeField, Min(0f)] float functionDuration = 1f, transitionDuration = 1f;


    float duration;
    bool transitioning;
    FunctionLibrary.FunctionName transitionFunction;

    ComputeBuffer positionBuffer;
    [SerializeField] ComputeShader computeShader;
    static readonly int positionId = Shader.PropertyToID("_Positions"),
                        resolutionId = Shader.PropertyToID("_Resolution"),
                        stepId = Shader.PropertyToID("_Step"),
                        timeId = Shader.PropertyToID("_Time"),
                        transitionProgressId = Shader.PropertyToID("_TransitionProgress");

    //Visualization no gameObjects:
    [SerializeField] Material material;
    [SerializeField] Mesh mesh;


    private void OnEnable()
    {
        positionBuffer = new ComputeBuffer(maxResolution * maxResolution, 3 * 4);
    }

    private void OnDisable()
    {
        positionBuffer.Release();
        positionBuffer = null;
    }

    void UpdateFunctionOnGPU()
    {
        float step = 2f / resolution;

        //Setting Values to Compute Shader:
        computeShader.SetInt(resolutionId, resolution);
        computeShader.SetFloat(stepId, step);

        computeShader.SetFloat(timeId, Time.time);
        if (transitioning)
        {
            computeShader.SetFloat(transitionProgressId, Mathf.SmoothStep(0f, 1f, duration / transitionDuration));
        }



        var kernelIndex = (int)function + (int)(transitioning ? transitionFunction : function) * FunctionLibrary.FunctionCount;
        computeShader.SetBuffer(kernelIndex, positionId, positionBuffer);

        //Running Kernel:
        //[0] = kernel index
        //[1, 2, 3] = amount of groups to run
        int groups = Mathf.CeilToInt(resolution / 8f);
        computeShader.Dispatch(kernelIndex, groups, groups, 1);

        material.SetBuffer(positionId, positionBuffer);
        material.SetFloat(stepId, step);

        var bounds = new Bounds(Vector3.zero, Vector3.one * (2f + 2f / resolution));
        //(Mesh, SubMesh(s), Material)
        Graphics.DrawMeshInstancedProcedural(mesh, 0, material, bounds, resolution * resolution);
    }

    private void Update()
    {
        duration += Time.deltaTime;

        if (transitioning)
        {
            if (duration >= functionDuration)
            {
                duration -= transitionDuration;
                transitioning = false;
            }
        }
        else if (duration >= functionDuration)
        {
            duration -= functionDuration;
            transitioning = true;
            transitionFunction = function;
            PickNextFunction();
        }

        UpdateFunctionOnGPU();
    }

    void PickNextFunction()
    {
        function = transitionMode == TransitionMode.Cycle ?
            FunctionLibrary.GetNextFunctionName(function) :
            FunctionLibrary.GetRandomFunctionName(function);
    }
}
