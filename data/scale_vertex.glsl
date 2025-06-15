#version 330 core
in vec4 al_pos;
in vec4 al_color;
in vec2 al_texcoord;
uniform mat4 al_projview_matrix;
out vec4 varying_color;
out vec2 varying_texcoord;

void main()
{
   varying_color = al_color;
   varying_texcoord = al_texcoord;
   gl_Position = al_projview_matrix * al_pos;
}

