#version 120

#pragma import_defines(CAST_SHADOW)

uniform sampler2D diffuse_map;
uniform sampler2D normal_map;

varying vec3 vertex;
varying mat3 tbn_matrix;

void main(void)
{
    vec4 color = texture2D(diffuse_map, gl_TexCoord[0].xy);
    if(color.w < 0.495f)
    {
        discard;
    }

#ifdef CAST_SHADOW
    gl_FragColor = vec4(1);
    return;
#endif

    vec3 n = texture2D(normal_map, gl_TexCoord[0].xy).xyz * 2 - 1;
    n = normalize(gl_NormalMatrix * tbn_matrix * n);
    // gl_FragColor = vec4(n * 0.5 + 0.5, 1);
    // gl_FragColor = vec4(tbn_matrix[2] * 0.5 + 0.5, 1);
    // return;

    vec3 l = normalize(gl_LightSource[1].position.xyz);
    vec3 v = normalize(-vertex);
    vec3 h = normalize(l + v);
    float ndotl = dot(l, n);

    // light back side
    if(ndotl < 0)
    {
        ndotl = -ndotl * 0.85f;
    }

    vec4 diffuse = gl_LightModel.ambient +
                   gl_LightSource[1].ambient * gl_FrontMaterial.ambient +
                   ndotl * gl_FrontMaterial.diffuse;

    gl_FragColor = color * diffuse;
}
