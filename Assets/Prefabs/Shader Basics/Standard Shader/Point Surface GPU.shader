    Shader "Graph/Point Surface GPU"
{

    Properties
    {
        _Smoothness ("Smoothness", Range(0,1)) = 0.5
    }

    SubShader
    {
        //Start CG PROGRAM:
        CGPROGRAM

        //Generate Surface Shader with standard lighting and full support for shadows
        #pragma surface ConfigureSurface Standard //fullforwardshadows addshadow

        //Procedural Rendering
        #pragma instancing_options assumeuniformscaling procedural:ConfigureProcedural

        //Force Unity to stall and immediately compile shaders right before it gets used for the first time
        #pragma editor_sync_compilation

        //Set Minimum for the shader's target level and quality
        #pragma target 4.5

        //Include 
        #include "PointGPU.hlsl"

        //Holds the Vector3 of the point in space:
        struct Input
        {
            float3 worldPos;
        };

        float _Smoothness;

	    void ConfigureSurface (Input input, inout SurfaceOutputStandard surface)
        {
            surface.Albedo = (input.worldPos * 0.5) + 0.5;
            surface.Smoothness = _Smoothness;
        }

        ENDCG
    }

    Fallback "Diffuse"
}
