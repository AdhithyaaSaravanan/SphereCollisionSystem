#version 410 core 

layout (location=0) in vec3 inVert;
layout (location=1) in vec3 normal;
uniform mat4 MVP;
out vec3 n;
void main()
{
  n=normal;
  
  gl_Position=MVP*vec4(inVert,1.0);
}