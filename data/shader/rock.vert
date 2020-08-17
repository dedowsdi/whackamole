#version 120

varying vec3 vertex;
varying mat3 tbn_matrix;

void main(void)
{
    vertex = (gl_ModelViewMatrix * gl_Vertex).xyz;

    gl_Position = gl_ProjectionMatrix * vec4(vertex, 1);
    gl_TexCoord[0] = gl_MultiTexCoord0;

    vec3 tangent = gl_MultiTexCoord1.xyz;
    vec3 bitangent = cross(gl_Normal, tangent);
    tbn_matrix = mat3(tangent, bitangent, gl_Normal);
}
