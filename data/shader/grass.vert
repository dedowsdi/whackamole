#version 120

#define MAX_EXPLOSIONS 8
#define MAX_DISTANCE 32

uniform vec4 explosions[MAX_EXPLOSIONS];  // view space

void main(void)
{
    vec4 vertex = gl_ModelViewMatrix * gl_Vertex;

    if (gl_MultiTexCoord0.t > 0.5)
    {
        for (int i = 0; i < MAX_EXPLOSIONS; i++)
        {
            float force = explosions[i].w;
            if (force <= 0)
            {
                continue;
            }

            vec3 pos = explosions[i].xyz;
            float dist = length(pos - vertex.xyz);
            force *= pow(clamp(1 - dist / MAX_DISTANCE, 0, 1), 0.35);
            vertex.xyz += normalize(vertex.xyz - pos) * force;
        }
    }

    gl_Position = gl_ProjectionMatrix * vertex;
    gl_TexCoord[0] = gl_MultiTexCoord0;
}
