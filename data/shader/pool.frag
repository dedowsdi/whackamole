#version 120

uniform sampler2D reflect_map;
uniform sampler2D refract_map;
uniform sampler2D depth_map;
uniform sampler2D dudv_map;
uniform sampler2D normal_map;

uniform vec2 render_target_size;
uniform float wave_strength = 0.02;
uniform float osg_SimulationTime;

varying vec3 normal;
varying vec3 vertex;

void main(void)
{
    // animate distortion
    float animStep = osg_SimulationTime * 0.02;
    vec2 st = gl_TexCoord[0].xy;
    vec2 distort_st = texture2D(dudv_map, vec2(st.x + animStep, st.y)).xy * 0.1;
    st += vec2(distort_st.x, distort_st.y + animStep);
    vec2 distortion = (texture2D(dudv_map, st) * 2 - 1).xy * wave_strength;

    // get refract and distorted reflect color
    vec2 colorTexcoord = clamp(gl_FragCoord.xy / render_target_size + distortion, 0, 1);
    vec4 reflect_color = texture2D(reflect_map, colorTexcoord);
    vec4 refract_color = texture2D(refract_map, colorTexcoord);

    // fresnel effect, use fixed normal
    vec3 n = normalize(gl_NormalMatrix * vec3(0, 0, 1));
    vec3 v = normalize(-vertex);
    float fresnel = clamp(dot(n, v), 0.15, 0.85);
    vec4 color = mix(reflect_color, refract_color, pow(fresnel, 2));

    // specular, use normal map normal
    vec3 l = normalize(gl_LightSource[1].position.xyz);
    vec2 normal_st = clamp(gl_TexCoord[0].xy + distortion, 0, 1);
    vec3 detail_normal =
        normalize(gl_NormalMatrix * (texture2D(normal_map, normal_st).xyz * 2 - 1));
    float ndotl = dot(l, detail_normal);
    if(ndotl > 0)
    {
        // color += ndotl * gl_FrontMaterial.diffuse;
        // specular
        vec3 h = normalize(l + v);
        float ndoth = clamp(dot(detail_normal, h), 0, 1);
        color += pow(ndoth, gl_FrontMaterial.shininess) *
                    gl_FrontMaterial.specular* gl_LightSource[1].specular;
    }

    gl_FragColor = color;
}
