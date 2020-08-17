#version 120

#define PI 3.1415926535897932384626433832795

uniform sampler2D diffuse_map;
uniform sampler2D normal_map;

varying vec3 vertex;
varying mat3 tbn_matrix;

#pragma import_defines(SHADOWED_SCENE)
#if SHADOWED_SCENE == 1
uniform sampler2DShadow osgShadow_shadowTexture;
uniform mat4 shadow_matrix;

uniform vec2 osgShadow_ambientBias;
uniform vec2 shadow_resolution = vec2(4096);
#endif

#pragma import_defines(MAX_LIGHTS)
#ifndef MAX_LIGHTS
#    define MAX_LIGHTS 2
#endif  // ifndef MAX_LIGHTS

// assume vertex normalized
// assume normal normalized
// assume light position nomalized
gl_LightProducts directionalLight(vec3 lvertex, vec3 normal, int light_index)
{
    gl_LightProducts lp;
    gl_LightSourceParameters light = gl_LightSource[light_index];

    // ambient
    lp.ambient = light.ambient * gl_FrontMaterial.ambient;

    // diffuse
    vec3 l = light.position.xyz;
    vec3 v = -lvertex;
    vec3 h = normalize(l + v);
    float ndotl = clamp(dot(l, normal), 0, 1);
    lp.diffuse = ndotl * light.diffuse * gl_FrontMaterial.diffuse;

    // specular
    if (ndotl > 0)
    {
        float ndoth = clamp(dot(normal, h), 0, 1);
        lp.specular = pow(ndoth, gl_FrontMaterial.shininess) * gl_FrontMaterial.specular *
                      light.specular;
    }

    return lp;
}

gl_LightProducts defaultLight(vec3 lvertex, vec3 normal)
{
    gl_LightProducts lp;
    lp.ambient = vec4(0);
    lp.diffuse = vec4(0);
    lp.specular = vec4(0);

    for (int i = 0; i < MAX_LIGHTS; i++)
    {
        gl_LightProducts p = directionalLight(lvertex, normal, i);
        lp.ambient += p.ambient;
        lp.diffuse += p.diffuse;
        lp.specular += p.specular;
    }

    return lp;
}

void main(void)
{
    vec3 n = texture2D(normal_map, gl_TexCoord[0].xy).xyz * 2 - 1;
    n = normalize(gl_NormalMatrix * tbn_matrix * n);
    gl_LightProducts lp = defaultLight(normalize(vertex), n);
    gl_FragColor = (lp.diffuse + lp.ambient) * texture2D(diffuse_map, gl_TexCoord[0].xy) + lp.specular;
    gl_FragColor.w = gl_FrontMaterial.diffuse.w;

#if SHADOWED_SCENE == 1
    float ndotl = dot(gl_LightSource[1].position.xyz, n);
    float bias = 0.003 * tan(acos(clamp(ndotl, 0, 1)));
    vec4 shadow_st = shadow_matrix * vec4(vertex, 1);
    float visibility = shadow2DProj(
        osgShadow_shadowTexture, vec4(shadow_st.xy, shadow_st.z - bias, shadow_st.w))
                           .x;

    gl_FragColor.xyz *= (osgShadow_ambientBias.x + osgShadow_ambientBias.y * visibility);
#endif
}
