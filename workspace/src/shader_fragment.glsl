#version 330 core

// Atributos de fragmentos recebidos como entrada ("in") pelo Fragment Shader.
// Neste exemplo, este atributo foi gerado pelo rasterizador como a
// interpolação da posição global e a normal de cada vértice, definidas em
// "shader_vertex.glsl" e "main.cpp".
in vec4 position_world;
in vec4 normal;

// Posição do vértice atual no sistema de coordenadas local do modelo.
in vec4 position_model;
 
// Coordenadas de textura obtidas do arquivo OBJ (se existirem!)
in vec2 texcoords;

// Matrizes computadas no código C++ e enviadas para a GPU
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Identificador que define qual objeto está sendo desenhado no momento
#define SPHERE 0
#define BOULDER  1
#define PLATFORM  2
#define BIRD  3
#define CHARACTER 4
#define MARIO_HAT 5
#define MARIO_PANTS 6
#define MARIO_FACE 7
#define MARIO_EYE 8
#define MARIO_GLOVE 9
#define MARIO_CLOTHES 10
#define MARIO_SHOES 11
#define MARIO_HAIR 12


uniform int object_id;

// Parâmetros da axis-aligned bounding box (AABB) do modelo
uniform vec4 bbox_min;
uniform vec4 bbox_max;

// Variáveis para acesso das imagens de textura

// Mario Hat Texture
uniform sampler2D TextureImage0;

// Mario Pants Texture
uniform sampler2D TextureImage1;

// Mario Face Texture
uniform sampler2D TextureImage2;

// Mario Eye
uniform sampler2D TextureImage3;

// Mario Gloves
uniform sampler2D TextureImage4;

// Mario Clothes
uniform sampler2D TextureImage5;

// Mario Shoes
uniform sampler2D TextureImage6;

// Mario Hair
uniform sampler2D TextureImage7;

// Grass
uniform sampler2D TextureImageGrass;

// GrassSide
uniform sampler2D TextureImageGrassSide;

// Dirt
uniform sampler2D TextureImageDirt;

// Blue Bird 
uniform sampler2D TextureImageBlueBird;


// O valor de saída ("out") de um Fragment Shader é a cor final do fragmento.
out vec4 color;

// Constantes
#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923

void main()
{
    // Obtemos a posição da câmera utilizando a inversa da matriz que define o
    // sistema de coordenadas da câmera.
    vec4 origin = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 camera_position = inverse(view) * origin;

    // O fragmento atual é coberto por um ponto que percente à superfície de um
    // dos objetos virtuais da cena. Este ponto, p, possui uma posição no
    // sistema de coordenadas global (World coordinates). Esta posição é obtida
    // através da interpolação, feita pelo rasterizador, da posição de cada
    // vértice.
    vec4 p = position_world;

    // Normal do fragmento atual, interpolada pelo rasterizador a partir das
    // normais de cada vértice.
    vec4 n = normalize(normal);

    // Vetor que define o sentido da fonte de luz em relação ao ponto atual.
    vec4 l = normalize(vec4(1.0,1.0,0.0,0.0));

    // Vetor que define o sentido da câmera em relação ao ponto atual.
    vec4 v = normalize(camera_position - p);

    float n_dot_l = dot(n,l);

    vec4 r = -l + 2*n*n_dot_l;
    float r_dot_v = v.x * r.x + v.y * r.y + v.z * r.z;

    // Coordenadas de textura U e V
    float U = 0.0;
    float V = 0.0;

    vec3 Kd; // Refletância difusa
    vec3 Ks; // Refletância especular
    vec3 Ka; // Refletância ambiente
    float q; // Expoente especular para o modelo de iluminação de Phong

    if ( object_id == SPHERE )
    {
        // PREENCHA AQUI as coordenadas de textura da esfera, computadas com
        // projeção esférica EM COORDENADAS DO MODELO. Utilize como referência
        // o slides 134-150 do documento Aula_20_Mapeamento_de_Texturas.pdf.
        // A esfera que define a projeção deve estar centrada na posição
        // "bbox_center" definida abaixo.

        vec4 bbox_center = (bbox_min + bbox_max) / 2.0;

        // COMO USEI LENGTH E O PORQUE DE VARIAVEIS OMISSAS:
        // 
        // p_linha = bbox_center + (ro * normalize(position_model - bbox_center));
        // p_vec = p_linha - bbox_center;
        //
        // Equivale a:
        // 
        // p_vec = ro * normalize(position_model - bbox_center);
        //
        // E se temos ro = length(position_model - bbox_center);
        // então:
        //        ro * normalize(position_model - bbox_center)
        //        =
        //        position_model - bbox_center
        //

        float ro = length(position_model - bbox_center);

        vec4 p_vec = position_model - bbox_center;

        float theta = atan(p_vec.x, p_vec.z);    // ou x/z ?
        float phi = asin(p_vec.y/ro);

        U = (theta + M_PI) / (2 * M_PI);
        V = (phi + M_PI_2) / M_PI;
    }
    else if ( object_id == BOULDER )
    {
        // PREENCHA AQUI as coordenadas de textura do coelho, computadas com
        // projeção planar XY em COORDENADAS DO MODELO. Utilize como referência
        // o slides 99-104 do documento Aula_20_Mapeamento_de_Texturas.pdf,
        // e também use as variáveis min*/max* definidas abaixo para normalizar
        // as coordenadas de textura U e V dentro do intervalo [0,1]. Para
        // tanto, veja por exemplo o mapeamento da variável 'p_v' utilizando
        // 'h' no slides 158-160 do documento Aula_20_Mapeamento_de_Texturas.pdf.
        // Veja também a Questão 4 do Questionário 4 no Moodle.

        float minx = bbox_min.x;
        float maxx = bbox_max.x;

        float miny = bbox_min.y;
        float maxy = bbox_max.y;

        float minz = bbox_min.z;
        float maxz = bbox_max.z;

        U = (position_model.x - minx) / (maxx - minx);
        V = (position_model.y - miny) / (maxy - miny);
    }
    else if ( object_id == PLATFORM )
    {

     Kd = vec3(0.4, 0.9, 0.5);
     Ks = vec3(0.4, 0.4, 0.4);
     Ka = Kd/2;
     q = 8.0;

    }
    else if ( object_id == BIRD )
    {
        Kd = vec3(0.4, 0.4, 0.8);
        Ks = vec3(0.8, 0.8, 0.8);
        Ka = Kd/2;
        q = 32.0;
    }

    else if ( object_id == CHARACTER)
    {
        Kd = vec3(0.4, 0.4, 0.8);
        Ks = vec3(0.8, 0.8, 0.8);
        Ka = Kd/2;
        q = 32.0;
    }
    else {
        Kd = vec3(0.08, 0.4, 0.8);
        Ks = vec3(0.8, 0.8, 0.8);
        Ka = Kd/2;
        q = 32.0;
    }


    if(object_id == MARIO_HAT){
        vec3 Kd_mario = texture(TextureImage0, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    else if(object_id == MARIO_PANTS){
        vec3 Kd_mario = texture(TextureImage1, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    else if(object_id == MARIO_FACE){
        vec3 Kd_mario = texture(TextureImage2, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }
    
    else if(object_id == MARIO_EYE){
        vec3 Kd_mario = texture(TextureImage3, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    else if(object_id == MARIO_GLOVE){
        vec3 Kd_mario = texture(TextureImage4, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    else if(object_id == MARIO_CLOTHES){
        vec3 Kd_mario = texture(TextureImage5, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    else if(object_id == MARIO_SHOES){
        vec3 Kd_mario = texture(TextureImage6, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    else if(object_id == MARIO_HAIR){
        vec3 Kd_mario = texture(TextureImage7, texcoords).rgb;
        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd_mario * (lambert + 0.01);
    }

    //else if(object_id == BIRD){
    //    float lambert = max(0, n_dot_l);

     //   color.rgb = texture(TextureImageBlueBird, texcoords).rgb;
    //}

    else if ( object_id == PLATFORM )
    {

    
       vec4 abs_normal = abs(normal);
       vec3 Kd_grass;
       float lambert = max(0, n_dot_l);



    
       if (abs_normal.y >= abs_normal.x && abs_normal.y >= abs_normal.z) 
        {
            if (normal.y > 0.0) {
                // Face de Cima (Topo)
                Kd_grass = texture(TextureImageGrass, vec2(position_model.x, position_model.z)).rgb;
            } else {
                // Face de Baixo (Fundo)
                Kd_grass =  texture(TextureImageDirt, vec2(position_model.x, position_model.z)).rgb;
            }

        }

        else{
            float miny = bbox_min.y + 1.0f;
            float maxy = bbox_max.y + 1.0f;

            V = ((position_model.y - miny) / (maxy - miny));

            if (abs_normal.x >= abs_normal.z)
            {
                Kd_grass = texture(TextureImageGrassSide, vec2(position_model.z, V)).rgb;
            }

            else{
                Kd_grass = texture(TextureImageGrassSide, vec2(position_model.x, V)).rgb;
            }

        
       }

            color.rgb = Kd_grass * (lambert + 0.01);

    }

    else if (object_id < 2) {
        // Obtemos a refletância difusa a partir da leitura da imagem TextureImage0
        vec3 Kd0 = texture(TextureImage0, vec2(U,V)).rgb;

        // Equação de Iluminação
        float lambert = max(0, n_dot_l);

        color.rgb = Kd0 * (lambert + 0.01);
    } else {
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


        color.rgb = lambert_diffuse_term + ambient_term + phong_specular_term;
    }

    

    // NOTE: Se você quiser fazer o rendering de objetos transparentes, é
    // necessário:
    // 1) Habilitar a operação de "blending" de OpenGL logo antes de realizar o
    //    desenho dos objetos transparentes, com os comandos abaixo no código C++:
    //      glEnable(GL_BLEND);
    //      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // 2) Realizar o desenho de todos objetos transparentes *após* ter desenhado
    //    todos os objetos opacos; e
    // 3) Realizar o desenho de objetos transparentes ordenados de acordo com
    //    suas distâncias para a câmera (desenhando primeiro objetos
    //    transparentes que estão mais longe da câmera).
    // Alpha default = 1 = 100% opaco = 0% transparente
    color.a = 1;

    // Cor final com correção gamma, considerando monitor sRGB.
    // Veja https://en.wikipedia.org/w/index.php?title=Gamma_correction&oldid=751281772#Windows.2C_Mac.2C_sRGB_and_TV.2Fvideo_standard_gammas
    color.rgb = pow(color.rgb, vec3(1.0,1.0,1.0)/2.2);
} 

