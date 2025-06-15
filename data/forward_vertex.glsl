#version 330 core
in vec4 al_pos;
in vec4 al_color;
in vec2 al_texcoord;
in vec3 al_user_attr_0;  // normal
in vec2 al_user_attr_1;  // uv2

uniform mat4 al_projview_matrix;
uniform mat4 model_matrix;

out vec3 varying_pos;
out vec3 varying_normal;
out vec2 varying_texcoord;
out vec2 varying_texcoord2;
out vec4 varying_color;

void main()
{
   varying_color = al_color;
   varying_texcoord = al_texcoord;
   varying_texcoord2 = al_user_attr_1;
   varying_pos = (model_matrix * al_pos).xyz;
   varying_normal = normalize(mat3(model_matrix) * al_user_attr_0);
   gl_Position = al_projview_matrix * al_pos;
}
