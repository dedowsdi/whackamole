#version 120

void main()
{
    vec2 st = 2 * gl_TexCoord[0].xy - 1;
    float l = dot(st, st);
    float f = 0.25 / pow(l, 0.67);
    f *= 1 - l / 1;
    gl_FragColor = vec4(f);
}
