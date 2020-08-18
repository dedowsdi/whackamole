#version 120

varying vec3 pos;

uniform sampler3D noise_map;
uniform vec4 sky_color = vec4(0.15, 0, 0, 0);
uniform vec4 waterline_color = vec4(0.15, 0.15, 0.3, 0);
uniform vec4 cloud_color = vec4(1, 1, 1, 1);
uniform vec3 moon = vec3(0, 0, 1);
uniform float osg_SimulationTime;

void main(void)
{
    vec3 p = normalize(pos);
    vec3 texcoord = p * 0.6;
    texcoord += osg_SimulationTime * 0.005;

    vec4 fbm = texture3D(noise_map, texcoord);
    float a = fbm.x;
    a += fbm.y;
    a += fbm.z;
    a += fbm.w;
    a = smoothstep(0, 1, a);

    // make it sparse
    a *= 1.52;
    a = pow(a, 3.57);

    // get alpha before minus. This will also give cloud bluish edge.
    float alpha = a;
    a -= 0.29;
    a = clamp(a, 0, 1);

    // light by moon
    float f =  clamp(dot(p, moon), 0, 1);
    f = 0.85 / pow(1-f, 0.15);
    f = clamp(f, 0, 1.5);
    a *= f;

    // sky gradient
    float b = dot(p, vec3(0, 0, 1));
    b = smoothstep(0, 1, b);
    vec4 gradient = mix(waterline_color, sky_color, b);

    vec4 color = mix(gradient, cloud_color, a);
    color.w = pow(alpha, 0.5);

    gl_FragColor = color;
}
