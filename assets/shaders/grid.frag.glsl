#version 460

layout(location = 0) out vec4 FragColor;
in vec2 uv;

uniform mat4 uInvViewProj;   // inverse(P * V)
uniform mat4 uViewProj;      // P * V
uniform float uNear;         // camera near
uniform float uFar;          // camera far

vec3 unproject(vec2 t, float z)
{
    vec4 p = uInvViewProj * vec4(t * 2.0 - 1.0, z, 1.0);
    return p.xyz / p.w;
}

// grid function per A Slice of Rendering: returns RGBA where A - line mask
vec4 gridColor(vec3 pos, float scale)
{
    // grid coords at given scale
    vec2 coord = pos.xz * scale;

    // derivatives for AA
    vec2 d = fwidth(coord);
    vec2 g = abs(fract(coord - 0.5) - 0.5) / d;     // distance in pixels to nearest line
    float line = 1.0 - min(min(g.x, g.y), 1.0);     // 1 on line, 0 outside

    vec3 col = vec3(0.2); // base gray

    return vec4(col, line);
}

void main()
{
    // ray through the pixel
    vec3 pNear = unproject(uv, -1.0);
    vec3 pFar = unproject(uv,  1.0);
    vec3 dir = normalize(pFar - pNear);

    // intersect with plane y = 0
    if (abs(dir.y) < 1e-6)
        discard;
    float t = -pNear.y / dir.y;
    if (t < 0.0)
        discard;

    vec3 pos = pNear + dir * t;

    // correct depth write
    vec4 clip = uViewProj * vec4(pos, 1.0);
    float ndcZ = clip.z / clip.w;
    float depth01 = ndcZ * 0.5 + 0.5;
    gl_FragDepth = clamp(depth01, 0.0, 1.0);

    // two resolutions summed: coarse and fine
    vec4 coarse = gridColor(pos, 0.1);          // 10-unit cells
    vec4 fine = gridColor(pos, 1.0);            // 1-unit cells

    // compose color and mask
    vec3 col = mix(vec3(0.0), coarse.rgb, coarse.a)
        + mix(vec3(0.0),   fine.rgb,   fine.a);
    float mask = clamp(coarse.a + fine.a, 0.0, 1.0);

    if (mask <= 0.0)
        discard;

    FragColor = vec4(col, 1.0);
}
