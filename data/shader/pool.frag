#version 120

// see
// https://www.youtube.com/watch?v=HusvGeEDU_U&list=PLRIWtICgwaX23jiqVByUs0bqhnalNTNZh&index=1
// for explanation

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

vec2 get_near_far(mat4 m)
{
    return vec2(m[3][2] / (m[2][2] - 1), m[3][2] / (m[2][2] + 1));
}

float frag_depth_to_perspective_depth(float frag_depth)
{
    float ndc_z = 2 * frag_depth - 1;
    vec2 nf = get_near_far(gl_ProjectionMatrix);
    return -2 * nf.x * nf.y / (ndc_z * (nf.y - nf.x) - nf.y - nf.x);
}

void main(void)
{
    vec2 st = gl_TexCoord[0].xy;
    vec2 proj_st = gl_FragCoord.xy / render_target_size;

    // pool depth is used to do something special with pool edge (glitches).
    float pool_depth = frag_depth_to_perspective_depth(texture2D(depth_map, proj_st).r) -
                       frag_depth_to_perspective_depth(gl_FragCoord.z);
    // gl_FragColor = vec4(pool_depth / 10);
    // return;

    // animate distortion, edge distortion should torward 0
    float animStep = osg_SimulationTime * 0.02;
    vec2 distort_st = texture2D(dudv_map, vec2(st.x + animStep, st.y)).xy * 0.06;
    distort_st = st + vec2(distort_st.x, distort_st.y + animStep);
    vec2 distortion = (texture2D(dudv_map, distort_st) * 2 - 1).xy * wave_strength *
                      pow(clamp(pool_depth / 10, 0, 1), 5);

    // get refract and reflect color
    vec2 color_st = clamp( proj_st + distortion, 0, 1);
    vec4 reflect_color = texture2D(reflect_map, color_st);
    vec4 refract_color = texture2D(refract_map, color_st);

    // fresnel effect, use fixed normal
    vec3 n = normalize(gl_NormalMatrix * vec3(0, 0, 1));
    vec3 v = normalize(-vertex);
    float fresnel = clamp(dot(n, v), 0.2, 0.8);
    vec4 color = clamp(mix(reflect_color, refract_color, pow(fresnel, 2)), 0, 1);

    // specular, use normal map normal
    vec3 l = normalize(gl_LightSource[1].position.xyz);
    vec2 normal_st = clamp(gl_TexCoord[0].xy + distortion, 0, 1);
    vec3 detail_normal =
        normalize(gl_NormalMatrix * (texture2D(normal_map, normal_st).xyz * 2 - 1));
    float ndotl = dot(l, detail_normal);
    if (ndotl > 0)
    {
        // specular
        vec3 h = normalize(l + v);
        float ndoth = clamp(dot(detail_normal, h), 0, 1);
        color += pow(ndoth, gl_FrontMaterial.shininess) * gl_FrontMaterial.specular *
                 gl_LightSource[1].specular * pow(clamp(pool_depth / 10, 0, 1), 2);
    }

    // give it some water color
    color *= vec4(0.5, 1, 1, 1);

    // alpha blend edge
    color.a = pow(clamp(pool_depth / 16, 0, 1), 1);

    gl_FragColor = color;
}
