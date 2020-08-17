#version 120

// adapted from http://www.evolved-software.com/shaders/lighting

#pragma import_defines(FLUTTER)

uniform float osg_SimulationTime;

// vertex_reverse_scale, strength_reverse_scale, variant_reverse_scale,
// flutter_reverse_scale
uniform vec4 wind_size = vec4(500, 150, 75, 0.5);

// force_scale, variant_scale
uniform vec3 wind_power = vec3(2, 4, 0.25);

varying vec3 vertex;
varying mat3 tbn_matrix;

vec3 applyWind(vec3 vertex) {
    // original algorithm use world vertex and normal to make difference betwen
    // trees, there aren't many trees in this game, they are placed in
    // different angle, so I use local instead.

    // tree_wind must wave slowly, otherwise user will see artificial movement
    vec3 tree_wind = vertex / wind_size.x + osg_SimulationTime * 0.2;
    float strength = 1 + abs(dot(cos(tree_wind), sin(tree_wind)));
    // This is the core part, make strength propotion to height.
    strength *= vertex.z / wind_size.y;

    // make sure vertex in the same direction have the same movement (will be scaled by
    // strength later)
    vec3 force = normalize(vertex) + osg_SimulationTime * 1.5;
    force.x = sin(force.x);
    force.y = cos(force.y);
    force.z = 0;

    vec3 animate = force * wind_power.x;

    // per vertex variant
    vec3 variant = normalize(vec3(vertex.x, vertex.y, vertex.z / wind_size.z)) +
                   osg_SimulationTime + gl_Color.y * 10;
    animate += cos(variant) * wind_power.y * pow(gl_Color.z, 2);

#ifdef FLUTTER
    // You must turn on double sided, otherwise you might see some leaves split into two
    vec3 flutter = (vertex / wind_size.w) + osg_SimulationTime * 15 + (gl_Color.y * 10);
    // require world normal? connect flutter and leaf angle
    flutter = abs(gl_Normal) * sin(dot(flutter, vec3(0.333f)));
    animate += (flutter * wind_power.z) * (gl_Color.x - 0.5);
#endif  // ifdef FLUTTER

    return animate * strength;
}

void main(void)
{
    vec3 pos = gl_Vertex.xyz + applyWind(gl_Vertex.xyz / gl_Vertex.w);
    vertex = (gl_ModelViewMatrix * vec4(pos, 1)).xyz;

    gl_Position = gl_ProjectionMatrix * vec4(vertex, 1);
    gl_TexCoord[0] = gl_MultiTexCoord0;

    vec3 tangent = gl_MultiTexCoord1.xyz;
    vec3 bitangent = cross(gl_Normal, tangent);
    tbn_matrix = mat3(tangent, bitangent, gl_Normal);
}
