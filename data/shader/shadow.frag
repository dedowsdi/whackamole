#version 120

#define PI 3.1415926535897932384626433832795

uniform sampler2D osgShadow_baseTexture;

#pragma import_defines(SHADOWED_SCENE)
#if SHADOWED_SCENE == 1
uniform sampler2DShadow osgShadow_shadowTexture;
uniform vec2 osgShadow_ambientBias;
uniform vec2 shadow_resolution = vec2(4096);
#endif

void main(void)
{
    gl_FragColor = gl_Color * texture2D(osgShadow_baseTexture, gl_TexCoord[0].xy);

#if SHADOWED_SCENE == 1
    // deal with shadow acne, bias is propotion to angle between light and normal
    float ndotl = dot(gl_LightSource[1].position.xyz, normalize(gl_TexCoord[2].xyz));
    float bias = 0.003 * tan(acos(clamp(ndotl, 0, 1)));

#    ifdef SHADOW_PCF
    const int pcfCount = 1;
    const float pcfTexels = (pcfCount * 2 + 1) * (pcfCount * 2 + 1);

    vec2 step = 1.0f / shadow_resolution;
    float total = 0;
    for (int i = -pcfCount; i <= pcfCount; ++i)
    {
        for (int j = -pcfCount; j <= pcfCount; ++j)
        {
            total += shadow2DProj(osgShadow_shadowTexture,
                gl_TexCoord[1] + vec4(step.x * i, step.y * j, 0, 0))
                         .x;
        }
    }
    float visibility = total / pcfTexels;
#    else
    float visibility = shadow2DProj(osgShadow_shadowTexture,
        vec4(gl_TexCoord[1].xy, gl_TexCoord[1].z - bias, gl_TexCoord[1].w))
                           .x;
#    endif

    gl_FragColor *= (osgShadow_ambientBias.x + visibility * osgShadow_ambientBias.y);
#endif
};
