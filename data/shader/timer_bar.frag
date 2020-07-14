#version 120

#define PI 3.1415926535897932384626433832795
#define border_size 2

uniform float percent = 0.5;
uniform vec2 size = vec2(200, 50);
uniform vec4 border_color = vec4(0.8);
uniform vec4 center_color = vec4(1, 0.5, 0, 1);

void main(void)
{
    vec2 st = gl_TexCoord[0].xy;
    vec2 p = st * size;

    if (p.x < border_size || p.x > (size.x - border_size) || p.y < border_size ||
        p.y > (size.y - border_size))
    {
        gl_FragColor = border_color;
    }
    else if ((p.x - border_size) / (size.x - 2 * border_size) < percent)
    {
        gl_FragColor = center_color;
    }
    else
    {
        discard;
    }
}
