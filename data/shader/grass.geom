#version 120

#extension GL_EXT_geometry_shader4 : enable

#define PI 3.1415926535897932384626433832795
#define TWO_PI (3.1415926535897932384626433832795 * 2)

#pragma import_defines(MAX_EXPLOSIONS)
#ifndef MAX_EXPLOSIONS
#    define MAX_EXPLOSIONS 8
#endif

#pragma import_defines(MAX_SWAY_DISTANCE)
#ifndef MAX_SWAY_DISTANCE
#    define MAX_SWAY_DISTANCE 32
#endif

uniform vec4 explosions[MAX_EXPLOSIONS];  // view space

uniform float size;

const vec2 tc0 = vec2(0);
const vec2 tc1 = vec2(1, 0);
const vec2 tc2 = vec2(1, 1);
const vec2 tc3 = vec2(0, 1);

void emit_vertex(vec3 pos, vec2 tc)
{
    if(tc.t > 0.5)
    {
        vec4 vertex = gl_ModelViewMatrix * vec4(pos, 1);
        for (int i = 0; i < MAX_EXPLOSIONS; i++)
        {
            float force = explosions[i].w;
            if (force <= 0)
            {
                continue;
            }

            vec3 pos = explosions[i].xyz;
            float dist = length(pos - vertex.xyz);
            force *= pow(clamp(1 - dist / MAX_SWAY_DISTANCE, 0, 1), 0.35);
            vertex.xyz += normalize(vertex.xyz - pos) * force;
        }
        gl_Position = gl_ProjectionMatrix * vertex;
    }
    else
    {
        gl_Position = gl_ModelViewProjectionMatrix * vec4(pos, 1);
    }

    gl_TexCoord[0].xy = tc;
    EmitVertex();
}

void main(void)
{
    vec3 vertex = gl_PositionIn[0].xyz;
    float hsize = size * 0.5;

    // create 3 rect as *
    for (int i = 0; i < 3; i++)
    {
        float angle = TWO_PI * i / 3;
        vec3 v0 = vec3(cos(angle) * hsize, sin(angle) * hsize, 0);
        vec3 v1 = vertex + vec3(-v0.x, -v0.y, 0);
        v0 += vertex;
        vec3 v2 = v1 + vec3(0, 0, size);
        vec3 v3 = v0 + vec3(0, 0, size);

        emit_vertex(v0, tc0);
        emit_vertex(v1, tc1);
        emit_vertex(v2, tc2);
        EndPrimitive();

        emit_vertex(v0, tc0);
        emit_vertex(v2, tc2);
        emit_vertex(v3, tc3);
        EndPrimitive();
    }
}
