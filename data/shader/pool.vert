#version 120

uniform float tiling = 0.5;

varying vec3 vertex;

#pragma import_defines(SHADOWED_SCENE)
#ifdef SHADOWED_SCENE
varying vec3 normal;
#endif

void main(void)
{
    vec4 v = gl_ModelViewMatrix * gl_Vertex;
    vertex = v.xyz;
    gl_Position = gl_ProjectionMatrix * v;
    gl_TexCoord[0] = gl_MultiTexCoord0 * tiling;
#ifdef SHADOWED_SCENE
    normal = gl_NormalMatrix * gl_Normal;
#endif
}
