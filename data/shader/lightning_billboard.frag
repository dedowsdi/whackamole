#version 120

#extension GL_EXT_geometry_shader4 : enable

varying vec2 texcoord;

uniform vec4 center_color = vec4(1);
uniform vec4 border_color = vec4(0, 0, 0, 1);
uniform float exponent = 0.25;

void main(void)
{
    vec2 v = texcoord - 0.5;
    float v_length2 = dot(v, v);

    // cull corner of end rect
    if (v_length2 > 0.25)
        discard;

    float f = 0.13f / pow(v_length2, 0.3);
    gl_FragColor = vec4(f);
}
