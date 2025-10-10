#version 460

const vec2 V[3] = vec2[3](
    vec2(-1.0, -1.0),
    vec2( 3.0, -1.0),
    vec2(-1.0,  3.0)
);

out vec2 uv;

void main()
{
    gl_Position = vec4(V[gl_VertexID], 0.0, 1.0);
    uv = gl_Position.xy * 0.5 + 0.5;
}
