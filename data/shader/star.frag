#version 120

varying float exponent;

void main()
{
    vec2 st = 2 * gl_TexCoord[0].xy - 1;
    float l = dot(st, st);
    float f = 0.25 / pow(l, exponent);
    f *= 1 - l / 1;
    gl_FragColor = vec4(f);
}
