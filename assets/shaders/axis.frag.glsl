#version 460

layout(location = 0) out vec4 FragColor;
in vec2 uv;

uniform mat4 uInvViewProj;   // inverse(P * V)
uniform mat4 uViewProj;      // P * V
uniform float uNear;
uniform float uFar;

vec3 unproject(vec2 t, float z)
{
    vec4 p = uInvViewProj * vec4(t * 2.0 - 1.0, z, 1.0);
    return p.xyz / p.w;
}

void orthoBasis(vec3 a, out vec3 b1, out vec3 b2)
{
    vec3 t = abs(a.y) < 0.999 ? vec3(0,1,0) : vec3(1,0,0);
    b1 = normalize(cross(a, t));
    b2 = normalize(cross(a, b1));
}

bool closestSegmentLine(vec3 p0, vec3 d, float tMax, vec3 a, out float dist, out vec3 q, out float tq)
{
    float c = dot(d, a);
    float rd = dot(d, p0);
    float ra = dot(a, p0);
    float denom = 1.0 - c*c;

    float s;
    if (denom < 1e-8)
    {
        s = ra;
        tq = 0.0;
    }
    else
    {
        s = (ra - c*rd) / denom;
        tq = c*s - rd;
        if (tq < 0.0)
            tq = 0.0;
        else if (tq > tMax)
            tq = tMax;
    }
    q = p0 + d * tq;
    vec3 onAxis = a * s;
    dist = length(q - onAxis);

    return (tq >= 0.0 && tq <= tMax + 1e-6);
}

struct AxisHit {
    float a;
    float depth01;
    vec3 col;
};

AxisHit evalAxis(vec3 pNear, vec3 dir, float tMax, vec3 axis, vec3 col, float thicknessPx)
{
    AxisHit H;
    H.a = 0.0;
    H.depth01 = 1.0;
    H.col = col;

    float dist;
    vec3 q;
    float tq;
    bool ok = closestSegmentLine(pNear, dir, tMax, axis, dist, q, tq);
    if (!ok)
        return H;

    vec3 b1, b2;
    orthoBasis(axis, b1, b2);

    float u = dot(q, b1);
    float v = dot(q, b2);
    vec2 duv = fwidth(vec2(u, v));
    float halfWidth = max(thicknessPx * 0.5 * length(duv), 1e-6);

    float a = 1.0 - smoothstep(halfWidth, 2.0*halfWidth, dist);
    if (a <= 0.0)
        return H;

    vec4 clip = uViewProj * vec4(q, 1.0);
    float ndcZ = clip.z / clip.w;
    H.depth01 = clamp(ndcZ * 0.5 + 0.5, 0.0, 1.0);
    H.a = a;
    return H;
}

void main()
{
    vec3 pNear = unproject(uv, -1.0);
    vec3 pFar = unproject(uv,  1.0);
    vec3 d = normalize(pFar - pNear);
    float tMax = length(pFar - pNear);

    const float thicknessPx = 1.0;

    AxisHit X = evalAxis(pNear, d, tMax, vec3(1,0,0), vec3(1,0,0), thicknessPx);
    AxisHit Y = evalAxis(pNear, d, tMax, vec3(0,1,0), vec3(0,1,0), thicknessPx);
    AxisHit Z = evalAxis(pNear, d, tMax, vec3(0,0,1), vec3(0,0,1), thicknessPx);

    float bestA = 0.0;
    float bestDepth = 1.0;
    vec3  bestCol = vec3(0.0);

    if (X.a > 0.0 && (X.depth01 <= bestDepth))
    {
        bestDepth = X.depth01;
        bestA = X.a;
        bestCol = X.col;
    }

    if (Y.a > 0.0 && (Y.depth01 <= bestDepth))
    {
        bestDepth = Y.depth01;
        bestA = Y.a;
        bestCol = Y.col;
    }

    if (Z.a > 0.0 && (Z.depth01 <= bestDepth))
    {
        bestDepth = Z.depth01;
        bestA = Z.a;
        bestCol = Z.col;
    }

    if (bestA <= 0.0)
        discard;

    gl_FragDepth = bestDepth;
    FragColor = vec4(bestCol, 1.0);
}
