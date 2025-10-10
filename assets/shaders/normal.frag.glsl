#version 330 core

in vec3 vNor;

uniform vec3 uLightDir;
uniform vec3 uColor;

out vec4 FragColor;

void main()
{
    vec3 N = normalize(vNor);
    float ndl = max(dot(N, normalize(uLightDir)), 0.0);
    float ambient = 0.2;
    vec3 lit = uColor * (ambient + (1.0 - ambient) * ndl);
    FragColor = vec4(lit, 1.0);
}
