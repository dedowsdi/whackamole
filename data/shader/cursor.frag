#version 120

uniform int flash = 1;
uniform float osg_SimulationTime;

#define BACKGROUND_COLOR vec4(0.7, 0.7, 0.0, 0.9);
#define BAR_COLOR vec4(1, 1, 1, 1);
#define BAR_HALF_SIZE 0.1
#define DURATION 0.5

void main(void)
{
    if (flash == 0)
    {
        gl_FragColor = BACKGROUND_COLOR;
        return;
    }

    vec2 st = gl_TexCoord[0].xy;
    float time = 1 - fract(osg_SimulationTime / DURATION);

    // animate bar, start when bar = 0, end when bar = 1
    float h = -BAR_HALF_SIZE + time * (1 + BAR_HALF_SIZE * 2);
    if (st.y > (h - BAR_HALF_SIZE) && st.y < (h + BAR_HALF_SIZE))
    {
        gl_FragColor = BAR_COLOR;
    }
    else
    {
        gl_FragColor = BACKGROUND_COLOR;
    }
}
