#version 120

uniform float render_target_scale = 1.0f;
varying float exponent;

void main(void)
{
    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1);
    exponent = gl_Vertex.w;

    gl_PointSize = 12 * render_target_scale;
}
