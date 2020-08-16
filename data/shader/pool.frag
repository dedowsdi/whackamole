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
uniform mat4 refract_projection_matrix;

varying vec3 vertex;
varying vec3 normal;

#pragma import_defines(SHADOWED_SCENE)
#if SHADOWED_SCENE == 1
uniform sampler2DShadow osgShadow_shadowTexture;
uniform mat4 shadow_matrix;

uniform vec2 osgShadow_ambientBias;
uniform vec2 shadow_resolution = vec2(4096);
#endif

vec2 get_near_far(mat4 m)
{
    return vec2(m[3][2] / (m[2][2] - 1), m[3][2] / (m[2][2] + 1));
}

float frag_depth_to_perspective_depth(float frag_depth, mat4 projection_matrix)
{
    float ndc_z = 2 * frag_depth - 1;
    vec2 nf = get_near_far(projection_matrix);
    return -2 * nf.x * nf.y / (ndc_z * (nf.y - nf.x) - nf.y - nf.x);
}

void main(void)
{
    vec2 st = gl_TexCoord[0].xy;
    vec2 proj_st = gl_FragCoord.xy / render_target_size;

    // pool depth is used to do something special with pool edge (glitches). Note refract
    // rtt use differnt projection matrix with current one.
    float pool_depth = frag_depth_to_perspective_depth(
                           texture2D(depth_map, proj_st).r, refract_projection_matrix) -
                       frag_depth_to_perspective_depth(gl_FragCoord.z, gl_ProjectionMatrix);
    // gl_FragColor = vec4(vec3(pool_depth/ 50 ), 1);
    // return;

    // animate distortion, edge distortion should torward 0
    float animStep = osg_SimulationTime * 0.015;
    vec2 distort_st = texture2D(dudv_map, vec2(st.x + animStep, st.y)).xy * 0.1;
    distort_st = st + vec2(distort_st.x, distort_st.y + animStep);
    vec2 distortion = (texture2D(dudv_map, distort_st) * 2 - 1).xy * wave_strength;
    distortion *= pow(clamp(pool_depth / 50, 0, 1), 2);

    // get refract and reflect color
    vec2 color_st = clamp( proj_st + distortion, 0, 1);
    vec4 reflect_color = texture2D(reflect_map, color_st);
    vec4 refract_color = texture2D(refract_map, color_st);

    // fresnel effect, use fixed normal
    vec3 n = normalize(normal);
    vec3 v = normalize(-vertex);
    float fresnel = clamp(dot(n, v), 0.2, 0.8);
    vec4 color = clamp(mix(reflect_color, refract_color, pow(fresnel, 1)), 0, 1);

    // specular, use normal map normal
    vec3 l = normalize(gl_LightSource[1].position.xyz);
    vec2 normal_st = gl_TexCoord[0].xy + distortion;
    vec3 detail_normal =
        normalize(gl_NormalMatrix * (texture2D(normal_map, normal_st).xyz * 2 - 1));
    float ndotl = dot(l, detail_normal);
    if (ndotl > 0)
    {
        // specular
        vec3 h = normalize(l + v);
        float ndoth = clamp(dot(detail_normal, h), 0, 1);
        vec4 specular = pow(ndoth, gl_FrontMaterial.shininess) * gl_FrontMaterial.specular *
                 gl_LightSource[1].specular;
        specular *= pow(clamp(pool_depth / 32, 0, 1), 3);
        color += specular;
    }

    // give it some water color
    color *= vec4(0.5, 1, 1, 1);

    // alpha blend edge
    color.a = pow(clamp(pool_depth / 32, 0, 1), 1);

    gl_FragColor = color;

#if SHADOWED_SCENE == 1
    float ndotgl = dot(gl_LightSource[1].position.xyz, n);
    float bias = 0.003 * tan(acos(clamp(ndotgl, 0, 1)));
    vec4 shadow_st = shadow_matrix * vec4(vertex, 1);
    float visibility = shadow2DProj(
        osgShadow_shadowTexture, vec4(shadow_st.xy, shadow_st.z - bias, shadow_st.w))
                           .x;

    gl_FragColor.xyz *= (osgShadow_ambientBias.x + osgShadow_ambientBias.y * visibility);
#endif

}
