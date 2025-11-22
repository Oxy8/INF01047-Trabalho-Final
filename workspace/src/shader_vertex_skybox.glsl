#version 330 core


// Vertex Shader (Skybox)
uniform mat4 view;      // Deve ser a matriz View SEM translação
uniform mat4 projection;
layout (location = 0) in vec3 position;
out vec3 tex_coords;

void main()
{
    tex_coords = position;
    // O W deve ser ajustado para garantir que a profundidade seja a máxima (fundo).
    // Usamos a view sem translação.
    vec4 pos = projection * view * vec4(position, 1.0);
    gl_Position = pos.xyww; // Garante que z = w (profundidade máxima)
}