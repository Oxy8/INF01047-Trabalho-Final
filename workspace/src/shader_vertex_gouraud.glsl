#version 330 core

// Atributos de vértice recebidos como entrada ("in") pelo Vertex Shader.
// Veja a função BuildTrianglesAndAddToVirtualScene() em "main.cpp".
layout (location = 0) in vec4 model_coefficients;
layout (location = 1) in vec4 normal_coefficients;
layout (location = 2) in vec2 texture_coefficients;

// Matrizes computadas no código C++ e enviadas para a GPU
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Atributos de vértice que serão gerados como saída ("out") pelo Vertex Shader.
// ** Estes serão interpolados pelo rasterizador! ** gerando, assim, valores
// para cada fragmento, os quais serão recebidos como entrada pelo Fragment
// Shader. Veja o arquivo "shader_fragment.glsl".

out vec4 vertex_color;

void main()
{
    gl_Position = projection * view * model * model_coefficients;
    
    vec4 origin = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 camera_position = inverse(view) * origin;

    // Posição do vértice atual no sistema de coordenadas global (World).
    vec4 position_world = model * model_coefficients;
    vec4 p = position_world;

    // Posição do vértice atual no sistema de coordenadas local do modelo.
    vec4 position_model = model_coefficients;

    // Normal do vértice atual no sistema de coordenadas global (World).
    // Veja slides 123-151 do documento Aula_07_Transformacoes_Geometricas_3D.pdf.
    vec4 normal = inverse(transpose(model)) * normal_coefficients;
    normal.w = 0.0;

    // Normal do fragmento atual, interpolada pelo rasterizador a partir das
    // normais de cada vértice.
    vec4 n = normalize(normal);

    // Vetor que define o sentido da fonte de luz em relação ao ponto atual.
    vec4 l = normalize(vec4(0.0,1.0,1.0,0.0));

    // Vetor que define o sentido da câmera em relação ao ponto atual.
    vec4 v = normalize(camera_position - p);

    float n_dot_l = dot(n,l);

    vec4 r = -l + 2*n*n_dot_l;
    float r_dot_v = v.x * r.x + v.y * r.y + v.z * r.z;

    vec3 Kd; // Refletância difusa
    vec3 Ks; // Refletância especular
    vec3 Ka; // Refletância ambiente
    float q; // Expoente especular para o modelo de iluminação de Phong


    Kd = vec3(0.08, 0.4, 0.8);
    Ks = vec3(0.8, 0.8, 0.8);
    Ka = Kd/2;
    q = 2;
    


    // Espectro da fonte de iluminação
    vec3 I = vec3(1.0,1.0,1.0); 

    // Termo difuso utilizando a lei dos cossenos de Lambert
    vec3 lambert_diffuse_term = Kd * I * max(0.0, n_dot_l); 

    // Espectro da luz ambiente
    vec3 Ia = vec3(0.2, 0.2, 0.2); // PREENCHA AQUI o espectro da luz ambiente

    // Termo ambiente
    vec3 ambient_term = Ka * Ia; // PREENCHA AQUI o termo ambiente

    // Termo especular utilizando o modelo de iluminação de Phong
    vec3 phong_specular_term  = Ks * I * pow(max(0.0, r_dot_v), q);

    vertex_color.rgb = lambert_diffuse_term + ambient_term + phong_specular_term;
    vertex_color.a = 1;
    vertex_color.rgb = pow(vertex_color.rgb, vec3(1.0,1.0,1.0)/2.2);

}

