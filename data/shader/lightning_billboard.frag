#version 120

#extension GL_EXT_geometry_shader4 : enable

varying vec2 texcoord;

uniform vec4 center_color = vec4(1);
uniform vec4 border_color = vec4(0, 0, 0, 1);
uniform float exponent = 0.37;

void main(void)
{
    vec2 v = texcoord - 0.5;
    float l = dot(v, v);

    // cull corner of end rect
    if (l > 0.25)
        discard;

    float f = 0.07f / pow(l, exponent);
    f *= pow(1 - sqrt(l) / 0.5, 0.49);
    gl_FragColor = vec4(f);
}
