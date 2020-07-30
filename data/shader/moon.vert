#version 120

uniform float render_target_scale = 1.0f;

void main(void)
{
    gl_Position = ftransform();
    gl_PointSize = 128 * render_target_scale;
}
