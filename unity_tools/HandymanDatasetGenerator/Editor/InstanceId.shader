Shader "Hidden/HandymanDataset/InstanceId"
{
    Properties { _IdColor ("Instance color", Color) = (0,0,0,1) _MaskEnabled ("Write mask", Float) = 1 _Cull ("Cull mode", Float) = 2 }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass
        {
            Tags { "LightMode"="SRPDefaultUnlit" }
            Cull [_Cull] ZWrite On ZTest LEqual Blend Off
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"
            float4 _IdColor;
            float _MaskEnabled;
            struct Varyings { float4 position : SV_POSITION; };
            Varyings vert(float4 position : POSITION)
            {
                Varyings o; o.position = UnityObjectToClipPos(position); return o;
            }
            float4 frag(Varyings i) : SV_Target { clip(_MaskEnabled - 0.5); return _IdColor; }
            ENDHLSL
        }
    }
}
