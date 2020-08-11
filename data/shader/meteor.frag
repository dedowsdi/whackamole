#version 120

uniform float aspect_ratio = 15;

void main()
{
    vec2 nst = gl_TexCoord[0].xy * 2 - 1;

    // move left until the round head touch left side
    float offset = 1 - 1 / aspect_ratio;
    vec2 st = vec2(nst.x + offset, nst.y);

    float max_height = 0.99;
    float smooth_begin = 0.6;
    float visibility = 0;

    if (st.x > 0)
    {
        // draw tail
        float height = mix(max_height, 0, st.x / (1 + offset));
        visibility = smoothstep(height, height * smooth_begin, abs(st.y));
    }
    else
    {
        // draw round head
        visibility = smoothstep(max_height, max_height * smooth_begin,
            length(vec2(st.x * aspect_ratio, st.y)));
    }

    // make it glow
    visibility *= 1 / pow(dot(st, st), 0.5);

    vec3 color = vec3(0.2, 0.1, 0.0) * visibility;

    gl_FragColor = vec4(color, visibility);
}
