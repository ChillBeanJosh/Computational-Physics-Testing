#if defined(UNITY_PROCEDURAL_INSTANCING_ENABLED)
StructuredBuffer<float2> _FluidInfo;
#endif

void GetFluidInfo_float(out float Type01, out float DensityRatio)
{
#if defined(UNITY_PROCEDURAL_INSTANCING_ENABLED)
        float2 info = _FluidInfo[unity_InstanceID];
        Type01 = info.x;
        DensityRatio = info.y;
#else
    Type01 = 0.0;
    DensityRatio = 1.0;
#endif
}

void GetFluidInfo_float_float(out float Type01, out float DensityRatio)
{
    GetFluidInfo_float(Type01, DensityRatio);
}

void GetFluidInfo_half_half(out half Type01, out half DensityRatio)
{
    float t, d;
    GetFluidInfo_float(t, d);
    Type01 = (half) t;
    DensityRatio = (half) d;
}