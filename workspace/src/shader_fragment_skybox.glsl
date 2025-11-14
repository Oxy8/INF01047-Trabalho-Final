#version 330 core

// Fragment Shader (Skybox)
in vec3 tex_coords;
uniform samplerCube skybox;

// O valor de saída ("out") de um Fragment Shader é a cor final do fragmento.
out vec4 color;

void main()
{
    color = texture(skybox, tex_coords);
}