#version 120

#extension GL_EXT_geometry_shader4 : enable

#define PI 3.1415926535897932384626433832795
#define TWO_PI (3.1415926535897932384626433832795 * 2)

#pragma import_defines(MAX_EXPLOSIONS)
#ifndef MAX_EXPLOSIONS
#    define MAX_EXPLOSIONS 8
#endif

#pragma import_defines(EXPLOSION_RADIUS)
#ifndef EXPLOSION_RADIUS
#    define EXPLOSION_RADIUS 32
#endif

#pragma import_defines(NUM_WINDS)
#ifndef NUM_WINDS
#    define NUM_WINDS 4
#endif

#pragma import_defines(SHADOWED_SCENE)

// world space
uniform vec4 explosions[MAX_EXPLOSIONS];

struct wind
{
    float amplitude;
    float frequence;
    float phi;
    float exponent;
    vec2 direction;
};

uniform wind winds[NUM_WINDS];
uniform float osg_SimulationTime;
uniform float size;
uniform float pool_radius = 150;

#ifdef SHADOWED_SCENE
varying vec3 vertex;
varying vec3 normal;
#endif

vec3 applyExplosions(vec3 pos)
{
    vec3 trans = vec3(0);
    for (int i = 0; i < MAX_EXPLOSIONS; i++)
    {
        float force = explosions[i].w;
        if (force <= 0)
        {
            continue;
        }

        vec3 exp_to_grass = pos - explosions[i].xyz;
        exp_to_grass.z = 0;
        float dist = length(exp_to_grass);
        force *= pow(clamp(1 - dist / EXPLOSION_RADIUS, 0, 1), 0.35);
        trans += exp_to_grass * force / dist;
    }
    return trans;
}

vec3 applyWinds(vec3 pos)
{
    vec3 trans = vec3(0);

    for (int i = 0; i < NUM_WINDS; i++)
    {
        wind w = winds[i];
        float inner = dot(w.direction, pos.xy) * w.frequence + osg_SimulationTime * w.phi;
        float base = (sin(inner) + 1) / 2;
        trans.xy += 2 * w.amplitude * pow(base, w.exponent) * w.direction;
    }

    return trans;
}


void emit_vertex(vec3 pos, vec2 tc)
{
    if(tc.t > 0.5)
    {
        pos += applyExplosions(pos) + applyWinds(pos);
    }

#ifdef SHADOWED_SCENE
    vertex = (gl_ModelViewMatrix * vec4(pos, 1)).xyz;
    normal = gl_NormalMatrix * vec3(0, 0, 1);
#endif

    gl_TexCoord[0].xy = tc;
    gl_Position = gl_ProjectionMatrix * vec4(vertex, 1);
    EmitVertex();
}

const vec2 tc0 = vec2(1, 0);
const vec2 tc1 = vec2(1, 1);
const vec2 tc2 = vec2(0, 1);
const vec2 tc3 = vec2(0, 0);
const float[] angles = float[](0, PI / 3, PI * 2 / 3);

void emit_rect(vec3 v0, vec3 v1, vec3 v2, vec3 v3)
{
    emit_vertex(v0, tc0);
    emit_vertex(v1, tc1);
    emit_vertex(v2, tc2);
    EndPrimitive();

    emit_vertex(v0, tc0);
    emit_vertex(v2, tc2);
    emit_vertex(v3, tc3);
    EndPrimitive();
}

void main(void)
{
    vec3 vertex = gl_PositionIn[0].xyz;
    float hsize = size * 0.5;
    float pool_radius2 = pool_radius * pool_radius;

    if (dot(vertex.xy, vertex.xy) < pool_radius2)
    {
        float theta = atan(vertex.y, vertex.x) + PI * 0.5f;

        // rotate 60, 120, create 2 half rect. This should be drawn first, when
        // you look from the origin, they are behind the rect that facing
        // center
        for (int i = 1; i < 3; i++)
        {
            float angle = theta - angles[i];
            vec3 v0 = vec3(cos(angle) * hsize, sin(angle) * hsize, 0);
            vec3 v3 = vertex + vec3(0);
            v0 += vertex;
            vec3 v1 = v0 + vec3(0, 0, size);
            vec3 v2 = v3 + vec3(0, 0, size);

            emit_rect(v0, v1, v2, v3);
        }

        // create 1 complete rect facing center
        vec3 v0 = vec3(cos(theta) * hsize, sin(theta) * hsize, 0);
        vec3 v3 = vertex + vec3(-v0.x, -v0.y, 0);
        v0 += vertex;
        vec3 v1 = v0 + vec3(0, 0, size);
        vec3 v2 = v3 + vec3(0, 0, size);

        emit_rect(v0, v1, v2, v3);

    }
    else
    {
        // create 3 rect as *
        for (int i = 0; i < 3; i++)
        {
            float angle = angles[i];
            vec3 v0 = vec3(cos(angle) * hsize, sin(angle) * hsize, 0);
            vec3 v3 = vertex + vec3(-v0.x, -v0.y, 0);
            v0 += vertex;
            vec3 v1 = v0 + vec3(0, 0, size);
            vec3 v2 = v3 + vec3(0, 0, size);

            emit_rect(v0, v1, v2, v3);
        }
    }
}
