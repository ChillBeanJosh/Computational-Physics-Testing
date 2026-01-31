    Shader "Graph/Point Surface"
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
        #pragma surface ConfigureSurface Standard fullforwardshadows

        //Set Minimum for the shader's target level and quality
        #pragma target 3.0

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
