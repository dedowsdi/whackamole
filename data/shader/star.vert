#version 120

varying float exponent;

void main(void)
{
    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1);
    exponent = gl_Vertex.w;
}
