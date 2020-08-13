#version 120

uniform sampler2D diffuse_map;

#pragma import_defines(SHADOWED_SCENE)
#ifdef SHADOWED_SCENE
uniform sampler2DShadow osgShadow_shadowTexture;
uniform mat4 shadow_matrix;

uniform vec2 osgShadow_ambientBias;
uniform vec2 shadow_resolution = vec2(4096);

varying vec3 vertex;
varying vec3 normal;
#endif

void main(void)
{
    vec4 color = texture2D(diffuse_map, gl_TexCoord[0].xy);
    gl_FragColor = color;

#ifdef SHADOWED_SCENE
    // deal with shadow acne, bias is propotion to angle between light and normal
    float ndotl = dot(gl_LightSource[1].position.xyz, normal);
    float bias = 0.003 * tan(acos(clamp(ndotl, 0, 1)));
    vec4 shadow_st = shadow_matrix * vec4(vertex, 1);
    float visibility = shadow2DProj(
        osgShadow_shadowTexture, vec4(shadow_st.xy, shadow_st.z - bias, shadow_st.w))
                           .x;
    gl_FragColor.xyz *= (osgShadow_ambientBias.x + osgShadow_ambientBias.y * visibility);
#endif  // ifdef SHADOWED_SCENE
}
