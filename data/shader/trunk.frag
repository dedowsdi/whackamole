#version 120

#define PI 3.1415926535897932384626433832795

uniform sampler2D diffuse_map;
uniform sampler2D normal_map;

varying vec3 vertex;
varying mat3 tbn_matrix;

void main(void)
{
    vec3 n = texture2D(normal_map, gl_TexCoord[0].xy).xyz * 2 - 1;
    n = normalize(gl_NormalMatrix * tbn_matrix * n);

    vec3 l = normalize(gl_LightSource[1].position.xyz);
    vec3 v = normalize(-vertex);
    vec3 h = normalize(l + v);
    float ndotl = clamp(dot(l, n), 0, 1);

    vec4 diffuse = gl_LightModel.ambient +
                   gl_LightSource[1].ambient * gl_FrontMaterial.ambient +
                   ndotl * gl_FrontMaterial.diffuse;

    vec4 color = texture2D(diffuse_map, gl_TexCoord[0].xy);
    gl_FragColor = color * diffuse;
}
