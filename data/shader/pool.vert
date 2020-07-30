#version 120

// varying vec4 clipVertex;

uniform float tiling = 0.5;

varying vec3 vertex;

void main(void)
{
    vec4 v = gl_ModelViewMatrix * gl_Vertex;
    vertex = v.xyz;
    gl_Position = gl_ProjectionMatrix * v;

    gl_TexCoord[0] = gl_MultiTexCoord0 * tiling;
}
