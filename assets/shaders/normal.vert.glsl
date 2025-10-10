#version 330 core

layout (location=0) in vec3 aPos;
layout (location=1) in vec3 aNor;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

out vec3 vNor;

void main()
{
    mat3 nmat = mat3(uModel);
    vNor = normalize(nmat * aNor);
    gl_Position = uProj * uView * uModel * vec4(aPos, 1.0);
}
