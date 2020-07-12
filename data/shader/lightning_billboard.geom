#version 120

#extension GL_EXT_geometry_shader4 : enable

// Create billboard along line, use line center as billboard center, you can
// define different BILLBOARD_TYPE to control how the side is calculated.
// All calculation done in view space.

// clang-format off
#pragma import_defines(BILLBOARD_TYPE)
// clang-format on

// use global billboard axis and 0,0,1 to calculate side. All billboards have
// the same side.
#define BT_GLOBAL 1

// use line and 0,0,1 to calculate side
#define BT_PER_LINE 2

// use line and -center to cauculate side
#define BT_PER_LINE_LOCAL 3

#ifndef BILLBOARD_TYPE
#    define BILLBOARD_TYPE BT_PER_LINE
#endif  // ifndef BILLBOARD_TYPE

#define PI 3.1415926535897932384626433832795
#define TWO_PI (3.1415926535897932384626433832795 * 2)

uniform float billboard_width = 2;

#if BILLBOARD_TYPE == BT_GLOBAL
uniform vec3 billboard_axis;
#endif

varying out vec2 texcoord;

void emit_vertex(vec4 pos, vec2 tc)
{
    gl_Position = pos;
    texcoord = tc;
    EmitVertex();
}

void main(void)
{
    float radius = billboard_width * 0.5f;

    // gl_in requires version 150
    vec3 p0 = (gl_ModelViewMatrix * gl_PositionIn[0]).xyz;
    vec3 p1 = (gl_ModelViewMatrix * gl_PositionIn[1]).xyz;
    vec3 v01 = normalize(p1 - p0);

#if BILLBOARD_TYPE == BT_PER_LINE_LOCAL
    vec3 ev = -0.5f * (p0 + p1);
#else
    vec3 ev = vec3(0, 0, 1);
#endif

#if BILLBOARD_TYPE == BT_GLOBAL
    // TODO pass in billboard_axis in view space
    vec3 side = radius * normalize(cross(mat3(gl_ModelViewMatrix) * billboard_axis, ev));
#else
    vec3 side = radius * normalize(cross(v01, ev));
#endif

    // triangle strip are created as:
    //  2  3
    //  0  1

    // get 4 corner of the center rect
    vec4 v0 = gl_ProjectionMatrix * vec4(p0 - side, 1);
    vec4 v1 = gl_ProjectionMatrix * vec4(p0 + side, 1);
    vec4 v2 = gl_ProjectionMatrix * vec4(p1 - side, 1);
    vec4 v3 = gl_ProjectionMatrix * vec4(p1 + side, 1);

    emit_vertex(v0, vec2(0, 0.5));
    emit_vertex(v1, vec2(1, 0.5));
    emit_vertex(v2, vec2(0, 0.5));
    emit_vertex(v3, vec2(1, 0.5));

    EndPrimitive();

    // close bottom gap with rect
    vec3 biside = v01 * radius;
    vec4 bv0 = gl_ProjectionMatrix * vec4(p0 - side - biside, 1);
    vec4 bv1 = gl_ProjectionMatrix * vec4(p0 + side - biside, 1);

    // texcoord is used to cull corner of gap rect, only semi circle is drawn.
    emit_vertex(bv0, vec2(0, 0));
    emit_vertex(bv1, vec2(1, 0));
    emit_vertex(v0, vec2(0, 0.5));
    emit_vertex(v1, vec2(1, 0.5));
    EndPrimitive();

    // close top gap with rect
    vec4 tv0 = gl_ProjectionMatrix * vec4(p1 - side + biside, 1);
    vec4 tv1 = gl_ProjectionMatrix * vec4(p1 + side + biside, 1);

    emit_vertex(v2, vec2(0, 0.5));
    emit_vertex(v3, vec2(1, 0.5));
    emit_vertex(tv0, vec2(0, 1));
    emit_vertex(tv1, vec2(1, 1));
    EndPrimitive();
}
