#include "matrices.h"
#include <optional>

// OBB = Oriented Bounding Box - caixa que envolve o objeto cijos eixos estão alinhados com os eixos locais do objeto, não necessariamente com os eixos globais
struct OBB {
    glm::vec3 center;      // centro da caixa
    glm::vec3 half_sizes;  // metade do comprimento em cada eixo local
    glm::mat3 orientation; // matriz 3x3 cujas colunas são os eixos locais (unitários)
};

const OBB createOBBFromAABB(const glm::vec3& aabb_min, const glm::vec3& aabb_max)
{
    OBB obb;
    obb.center = (aabb_min + aabb_max) / 2.0f;
    obb.half_sizes = (aabb_max - aabb_min) / 2.0f;
    obb.orientation = glm::mat3(1.0f); // inicialmente alinhada aos eixos globais
    return obb;
}

void updateOBB(OBB& obb, const glm::mat4& model, const glm::vec3& initial_center_local_coords, glm::vec3& initial_half_sizes)
{

    obb.orientation[0] = glm::normalize(glm::vec3(model[0]));
    obb.orientation[1] = glm::normalize(glm::vec3(model[1]));
    obb.orientation[2] = glm::normalize(glm::vec3(model[2]));

    obb.center = glm::vec3(model * glm::vec4(initial_center_local_coords, 1.0f)); 

    // Extrai escalas
    glm::vec3 scale;
    scale.x = glm::length(glm::vec3(model[0]));
    scale.y = glm::length(glm::vec3(model[1]));
    scale.z = glm::length(glm::vec3(model[2]));

    // Atualiza half-sizes (apenas escala afeta o tamanho)
    obb.half_sizes = initial_half_sizes * scale;


    //printf("Model[0] : %.2f %.2f %.2f %.2f\n", model[0][0], model[0][1], model[0][2], model[0][3]);
    //printf("Model[1] : %.2f %.2f %.2f %.2f\n", model[1][0], model[1][1], model[1][2], model[1][3]);
    //printf("Model[2] : %.2f %.2f %.2f %.2f\n", model[2][0], model[2][1], model[2][2], model[2][3]);
    //printf("Model[3] : %.2f %.2f %.2f %.2f\n", model[3][0], model[3][1], model[3][2], model[3][3]);     
}


// AABB = Axis Aligned Bounding Box - caixa que envolve o objeto cujos eixos estão alinhados com os eixos globais



// =====================================
// COLISÕES
// =====================================


const bool colision_aabb_plane(glm::vec3& aabb_min, const glm::vec3& aabb_max,
                              const glm::vec3& plane_normal, float plane_d)
{
    // Centro da AABB e semi-extensões
    glm::vec3 c = (aabb_min + aabb_max) / 2.0f;
    glm::vec3 e = aabb_max - c;

    // Computa o raio de projeção da AABB no plano
    float r = e.x * fabs(plane_normal.x) +
              e.y * fabs(plane_normal.y) +
              e.z * fabs(plane_normal.z);
    
    // Distância do centro da AABB ao plano
    float s = glm::dot(plane_normal, c) - plane_d;

    // A colisão ocorre se a distância do centro da AABB ao plano for menor que o raio de projeção, ou seja
    // se o intervalo [s-r,s+r] contiver o zero, o que significa que tem vértices da AABB em ambos os lados do plano

    return fabs(s) <= r;
}


const bool colision_aabb_aabb(glm::vec3& aabb1_min, const glm::vec3& aabb1_max,
                         glm::vec3& aabb2_min, const glm::vec3& aabb2_max)
{
    // Testa sobreposição em cada eixo
    bool overlap_x = (aabb1_min.x <= aabb2_max.x) && (aabb1_max.x >= aabb2_min.x);
    bool overlap_y = (aabb1_min.y <= aabb2_max.y) && (aabb1_max.y >= aabb2_min.y);
    bool overlap_z = (aabb1_min.z <= aabb2_max.z) && (aabb1_max.z >= aabb2_min.z);

    // Colisão ocorre se houver sobreposição em todos os eixos
    return overlap_x && overlap_y && overlap_z;
}


// SAT - Separating Axis Theorem
// 2 formas convexas não colidem se e somente se existir um eixo ao longo do qual suas projeções não se sobrepõem <-> UM PLANO DE SEPARAÇÃO ENTRE ELAS, DO QUAL ESSE EIXO É PERPENDICULAR.
// Para OBBs, os eixos a serem testados são os 3 eixos de cada OBB (6 no total) e os produtos vetoriais entre cada par de eixos (9 no total, vetores normais a ambos os eixos), totalizando 15 eixos a serem testados.


const float projectOBBOnAxis(const OBB& obb, const glm::vec3& axis)
{
    // Assume que o eixo já está normalizado

    // Projeta os 3 eixos locais da OBB no eixo de projeção - cada uma dessas projeções é a contribuição do respectivo eixo ao raio total nesse eixo de projeção
    float projection_x = fabs(glm::dot(axis, obb.orientation[0])) * obb.half_sizes.x;
    float projection_y = fabs(glm::dot(axis, obb.orientation[1])) * obb.half_sizes.y;
    float projection_z = fabs(glm::dot(axis, obb.orientation[2])) * obb.half_sizes.z;

    //  Logo, a soma das projeções nos 3 eixos dá o raio total da OBB nesse eixo de projeção
    return projection_x + projection_y + projection_z;
}


struct SeparationInfo {
    bool separated;     // existe um plano de separação?
    float penetration;  // se não, o quanto elas se sobrepõem
    bool norm_direction; // direção da normal (para qual OBB a normal aponta)
};

SeparationInfo existSeparatingPlane(const glm::vec3& vector_between_centers, const glm::vec3& axis, 
                                    const OBB& obb1, const OBB& obb2)
{
    // Projeta a distância entre os centros das OBBs no eixo
    float signed_distance = glm::dot(vector_between_centers, axis);
    float distance = fabs(signed_distance);

    bool norm_direction = (signed_distance > 0.0f);

    // Calcula a soma dos raios projetados das duas OBBs no mesmo eixo
    float radius1 = projectOBBOnAxis(obb1, axis);
    float radius2 = projectOBBOnAxis(obb2, axis);

    float totalRadius = radius1 + radius2;

    // Se a distância projetada entre os centros for maior que a soma dos raios,
    // significa que existe um espaço entre as projeções das OBBs nesse eixo,
    // logo, existe um plano de separação entre elas (não há colisão)
    SeparationInfo info;
    if (distance > totalRadius) {
        // existe separação
        info.separated = true;
        info.penetration = 0.0f;
    } else {
        // colidindo ao longo desse eixo
        info.separated = false;
        info.penetration = totalRadius - distance;
    }

    info.norm_direction = norm_direction;

    return info;
}


struct CollisionResult {
    bool colliding;
    glm::vec3 normal;     // eixo da menor penetração
    float penetration;    // profundidade da menor penetração
    glm::vec3 contact_point; // ponto de contato da colisão
};

typedef struct CollisionResult  CollisionResult;

CollisionResult colision_obb_obb(const OBB& obb1, const OBB& obb2)
{
    CollisionResult result;
    result.colliding = true;
    result.penetration = std::numeric_limits<float>::max();
    result.normal = glm::vec3(0.0f);

    // Vetor entre os centros das OBBs
    glm::vec3 vector_between_centers = obb2.center - obb1.center;

    // Testa cada um dos 15 eixos possíveis: os 3 eixos de cada OBB + 9 eixos normais às faces de uma e outra: 9 produtos vetoriais - cada eixo de uma OBB com cada eixo da outra OBB
    auto testAxis = [&](const glm::vec3& axis)
    {
        // Se o eixo for nulo (muito pequeno), não faz sentido testá-lo
        if (glm::length(axis) < 1e-6f) return;

        // Garantimos um eixo normalizado
        glm::vec3 normAxis = glm::normalize(axis);

        // Testa se existe um plano de separação ao longo desse eixo
        SeparationInfo info = existSeparatingPlane(vector_between_centers, normAxis, obb1, obb2);

        // Se existir um plano de separação, as OBBs não estão colidindo
        if (info.separated) {
            result.colliding = false;

        // Se não existir, pode ser que estejam (só se todos os eixos testados não tiverem planos de separação)
        // Nesse caso, guardamos a menor penetração (valor mínimo que precisamos mover o objeto no eixo correspondente para que não colida) e o eixo correspondente
        } else if (info.penetration < result.penetration) {
            result.penetration = info.penetration;
            result.normal = normAxis * (info.norm_direction ? -1.0f : 1.0f);
        }
    };

    // 3 eixos do obb1 + 3 do obb2
    for (int i = 0; i < 3; ++i) testAxis(obb1.orientation[i]);
    for (int i = 0; i < 3; ++i) testAxis(obb2.orientation[i]);

    // 9 eixos cruzados
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            testAxis(glm::cross(obb1.orientation[i], obb2.orientation[j]));

    return result;
}


CollisionResult colision_obb_aabb(const OBB& obb, const glm::vec3& aabb_min, const glm::vec3& aabb_max)
{
    // Primeiro, convertemos a AABB em OBB alinhado aos eixos globais
    OBB aabb_as_obb = createOBBFromAABB(aabb_min, aabb_max);

    // Depois, usamos a função de colisão entre OBBs
    return colision_obb_obb(obb, aabb_as_obb);
}



/**
 * @brief Tenta resolver a colisão entre o personagem e um objeto estático (plataforma).
 * * @param character_pos Posição do personagem (passada por referência para atualização).
 * @param character_vel Velocidade do personagem (passada por referência para correção).
 * @param grounded_flag Flag de estado (passada por referência).
 * @param falling_flag Flag de estado (passada por referência).
 * @param character_obb Bounding box orientada (OBB) do personagem.
 * @param platform_min Coordenada mínima da AABB da plataforma.
 * @param platform_max Coordenada máxima da AABB da plataforma.
 */
void resolve_collision_obb_aabb(
    glm::vec4& character_pos,
    glm::vec4& character_vel,
    bool& grounded_flag,
    const OBB& character_obb,
    const glm::vec3& platform_min,
    const glm::vec3& platform_max
)
{
    // 1. Detecção de Colisão
    CollisionResult colision = colision_obb_aabb(character_obb, platform_min, platform_max);

    if (colision.colliding) {
        
        // Corrige posição (Com Slop)
        float correction_amount = colision.penetration;

        if (correction_amount > 0.0f) {
             glm::vec4 normal_vec = glm::vec4(colision.normal, 0.0f);
             // Aplica a correção de posição (empurra para fora da colisão)
             character_pos += normal_vec * correction_amount;
        }

        // Corrige velocidade (Impulso de colisão)
        float vn = glm::dot(glm::vec3(character_vel), colision.normal);

        if (vn < 0.0f) {
             // Anula a componente de velocidade que está penetrando (a normal da colisão que detectamos)
             character_vel -= vn * glm::vec4(colision.normal, 0.0f);
        }
        
        // Atualiza o estado: Verificar se a colisão é um chão (normal vertical)
        if(colision.normal.y > 0.7f) { // Limiar de 0.7f (aprox. 45 graus)
             // Estabilização vertical: Zera a velocidade y no contato para evitar oscilação
             character_vel.y = 0.0f;
             grounded_flag = true;
        }

    }
    
    else {
        // Se não houver colisão, o objeto não está no chão
        grounded_flag = false;
    }
}


struct Sphere {
    glm::vec3 center;
    float radius;
    glm::vec4 velocity;
};

CollisionResult collision_sphere_obb(
    const Sphere& sphere,
    const OBB& obb, int vertical_priority) {
    // Leva o centro da esfera para as coordenadas locais da OBB
    // A matriz de orientação da OBB é ortonormal, então sua inversa é igual à sua transposta

    glm::mat3 obb_orientation_transpose = glm::transpose(obb.orientation);
    glm::vec3 C_local = obb_orientation_transpose * (sphere.center - obb.center);

   // printf("Sphere center world: %.2f %.2f %.2f\n", sphere.center.x, sphere.center.y, sphere.center.z);
   // printf("OBB center world: %.2f %.2f %.2f\n", obb.center.x, obb.center.y, obb.center.z);
   // printf("C_local: %.2f %.2f %.2f\n", C_local.x, C_local.y, C_local.z);



    // Agora pegamos o ponto mais próximo da OBB ao centro da esfera, EM COORDENADAS LOCAIS
    // Como está em coordenadas locais, a OBB está alinhada aos eixos, logo o ponto mais próximo é simplesmente o centro da esfera "clampado" às extents da OBB
    // Com "clampado" quer se dizer, para cada coordenada, se o valor do centro da esfera estiver fora do intervalo [-half_size, half_size], usamos o valor do limite mais próximo

    glm::vec3 closest;
    closest.x = glm::clamp(C_local.x, -obb.half_sizes.x, obb.half_sizes.x);
    closest.y = glm::clamp(C_local.y, -obb.half_sizes.y, obb.half_sizes.y);
    closest.z = glm::clamp(C_local.z, -obb.half_sizes.z, obb.half_sizes.z);

    

   // printf("Closest local: %.2f %.2f %.2f\n", closest.x, closest.y, closest.z);

    // Agora com esse ponto da OBB mais próximo da esfera em coordenadas locais, voltamos para coordenadas globais
    glm::vec3 closestWorld = obb.center + obb.orientation * closest;
   // printf("Closest world: %.2f %.2f %.2f\n", closestWorld.x, closestWorld.y, closestWorld.z);


    // Agora podemos calcular a distância entre esse ponto e o centro da esfera em coordenadas globais e ver se é menor que o raio da esfera
    // Aqui por otimização evitamos a raiz quadrada, comparando o quadrado das distâncias
    glm::vec3 diff = sphere.center - closestWorld;
    float dist_squared = glm::dot(diff, diff);


    CollisionResult result;
    result.colliding = false;
    result.penetration = 0.0f;
    result.normal = glm::vec3(0.0f);
    result.contact_point = glm::vec3(0.0f);


   // printf("Dist squared: %.2f, Radius squared: %.2f\n", dist_squared, sphere.radius * sphere.radius);

  // printf("Distance squared: %.6f, Radius squared: %.6f\n", dist_squared, sphere.radius * sphere.radius);
    // Se a distância ao quadrado for menor que o raio, há colisão
    if(dist_squared < sphere.radius * sphere.radius) {
        float dist = sqrt(dist_squared);

        // Calculamos o ponto mais próximo da borda da OBB ao centro da esfera
        float dx = obb.half_sizes.x - fabs(C_local.x);
        float dy = obb.half_sizes.y - fabs(C_local.y);
        float dz = obb.half_sizes.z - fabs(C_local.z);

        // Se dx for o menor, o ponto mais próximo é ao longo do eixo x local
        if(dx < dy && dx < dz) {
            closest.x = (C_local.x > 0 ? obb.half_sizes.x : -obb.half_sizes.x);
        }
        // Se dy for o menor, o ponto mais próximo é ao longo do eixo y local
        else if(dy < dz) {
            if(vertical_priority == 1)
                closest.y = obb.half_sizes.y;
            else if(vertical_priority == -1)
                closest.y = -obb.half_sizes.y;
            else
                closest.y = (C_local.y > 0 ? obb.half_sizes.y : -obb.half_sizes.y);
        }
        // Se dz for o menor, o ponto mais próximo é ao longo do eixo z local
        else {
            closest.z = (C_local.z > 0 ? obb.half_sizes.z : -obb.half_sizes.z);
        }


        closestWorld = obb.center + obb.orientation * closest;
        printf("Closest world (min penetration): %.2f %.2f %.2f\n", closestWorld.x, closestWorld.y, closestWorld.z);

        glm::vec3 normal;

        // Ainda tem um pedacinho da esfera fora da OBB e portanto a normal da colisão pode ser calculada normalmente
        if (dist > 1e-6f) {
         //   printf("Calculating normal normally\n");
            normal = diff / dist;
        } else {
        //    printf("Calculating normal with min penetration\n");
            // Esfera dentro da caixa -> não tem normal única -> precisamos escolher uma normal
            // Escolheremos o eixo com menor penetração -> o mínimo que tem ser movido para sair da colisão
            glm::vec3 local = C_local; // esfera no espaço da OBB
            glm::vec3 d = obb.half_sizes - glm::abs(local);


            // Escolhe o eixo com menor folga
            if (d.x < d.y && d.x < d.z)
                normal = glm::vec3((local.x > 0 ? 1.0f : -1.0f), 0, 0);
            else if (d.y < d.z){
                if(vertical_priority == 1)
                    normal = glm::vec3(0, 1.0f, 0);
                else if(vertical_priority == -1)
                    normal = glm::vec3(0, -1.0f, 0);
                else
                    normal = glm::vec3(0, (local.y > 0 ? 1.0f : -1.0f), 0);
            }
            else
                normal = glm::vec3(0, 0, (local.z > 0 ? 1.0f : -1.0f));

            normal = obb.orientation * normal; // volta para o espaço global
        }
        
       // printf("Collision normal: %.2f %.2f %.2f\n", normal.x, normal.y, normal.z);




        result.normal = normal;
        result.penetration = sphere.radius - dist;
        printf("Penetration 2: %.6f\n", result.penetration);
        result.colliding = true;
        result.contact_point = closestWorld;
    }

    return result;
}


void resolve_collision_sphere_obb(
    glm::vec4& sphere_center,
    glm::vec4& sphere_velocity,
    float sphere_radius,
    const OBB& obb,
    int vertical_priority = 0
) {

    Sphere sphere = { glm::vec3(sphere_center), sphere_radius, sphere_velocity };
    CollisionResult collision = collision_sphere_obb(sphere, obb, vertical_priority);

    if(collision.colliding) {
       // printf("Collision detected!\n");
        // Corrige posição
        float correction_amount = collision.penetration;
       // printf("Correction amount: %.6f\n", correction_amount);
        if(correction_amount > 0.0f) {
            glm::vec4 normal_vec = glm::vec4(collision.normal, 0.0f);
            printf("Normal vec: %.2f %.2f %.2f\n", normal_vec.x, normal_vec.y, normal_vec.z);
            printf("Contact point: %.2f %.2f %.2f\n", collision.contact_point.x, collision.contact_point.y, collision.contact_point.z);
            printf("Penetração da câmera: %f\n", correction_amount);

            
            sphere_center = glm::vec4(collision.contact_point, 1.0f) + glm::vec4(collision.normal, 0.0f) * (sphere_radius + 0.1f);
        }

        // Corrige velocidade
        // Isso é bem inútil por enquanto, já que a câmera não se move e os projéteis quando colidem simplesmente param. Mas fica aqui por enquanto caso acrescentemos algo como ricochete.
        // Só no caso do ricochete, teríamos que aplicar uma **fração** do impulso inverso, não anular completamente a componente da velocidade na direção da normal.
        // Essa fração dependeria de um coeficiente de restituição do material que poderia ser um parâmetro da função
        float vn = glm::dot(glm::vec3(sphere_velocity), collision.normal);
        if(vn < 0.0f) {
            sphere_velocity -= vn * glm::vec4(collision.normal, 0.0f);
        }
    }
}


/**
 * @brief Calcula a distância 't' (parâmetro escalar do raio) da primeira intersecção 
 * entre um raio e uma Oriented Bounding Box (OBB).
 *
 * O algoritmo funciona verificando a intersecção do raio com os três 'slabs' 
 * (fatias espaciais) que definem o OBB. A intersecção final é o intervalo 
 * de tempo (t_min a t_max) comum a todos os slabs.
 *
 * @param ray_origin A origem (posição inicial) do raio no espaço do mundo.
 * @param ray_dir A direção normalizada do raio.
 * @param obb_center O centro da OBB no espaço do mundo.
 * @param obb_axes Os três eixos ortonormais de orientação do OBB (matriz 3x3).
 * @param half_sizes As semi-extensões (metade da largura/altura/profundidade) do OBB.
 * @return float A distância 't' (parâmetro escalar do raio) da primeira intersecção. 
 * Retorna -1.0f se não houver intersecção válida.
 */
float intersectRayOBB(
    const glm::vec3& ray_origin,
    const glm::vec3& ray_dir,
    const glm::vec3& obb_center,
    const glm::mat3& obb_axes,    
    const glm::vec3& half_sizes
) {
    // Início do intervalo de intersecção (limite mais próximo)
    float t_min = 0.0f; 
    
    // Fim do intervalo de intersecção (limite mais distante)
    float t_max = 1e30f;          

    // Vetor do centro do OBB até a origem do raio (OBB_Center -> Ray_Origin).
    glm::vec3 p = obb_center - ray_origin; 

    // Percorre cada eixo do OBB (X, Y, Z no espaço local do OBB)
    for (int i = 0; i < 3; i++) {
        const glm::vec3& axis = obb_axes[i];
        
        // Distância projetada da origem do raio ao centro do OBB no eixo 'axis'
        float e = glm::dot(axis, p);
        
        // Projeção da direção do raio no eixo 'axis'. Usado como denominador
        float f = glm::dot(axis, ray_dir);

        const float EPS = 1e-6f;

        // Se o raio não for paralelo ao plano (não perpendicular ao eixo 'axis' <-> produto escalar com o eixo não é zero):
        if (fabs(f) > EPS) {
            
            // Calcula os dois pontos de intersecção do raio com os planos 
            // de 'slab' (faces paralelas) no eixo 'i'.
            // f (denominador) representa a velocidade de aproximação no eixo.
            float t1 = (e + half_sizes[i]) / f;
            float t2 = (e - half_sizes[i]) / f;

            // Ordena t1 e t2 para que t1 seja o ponto de entrada e t2 o de saída.
            if (t1 > t2) std::swap(t1, t2);

            // Atualiza o intervalo [t_min, t_max]:
            // t_min é o ponto de entrada mais distante de todos os slabs (MAX(t1))
            if (t1 > t_min) t_min = t1;
            // t_max é o ponto de saída mais próximo de todos os slabs (MIN(t2))
            if (t2 < t_max) t_max = t2;

            // Se o ponto de entrada mais distante (t_min) for maior que o ponto 
            // de saída mais próximo (t_max), os slabs não se sobrepõem: sem intersecção
            if (t_min > t_max) return -1.0f; 
        
        } else {
            // Raio paralelo ao plano (f aproximadamente 0)
            // Se o raio é paralelo, verifica se a origem (e + half_sizes[i] e e - half_sizes[i]) está fora 
            // do slab. Se estiver, não há intersecção, independentemente de t
            if (-e - half_sizes[i] > 0.0f || -e + half_sizes[i] < 0.0f)
                return -1.0f;
        }
    }

    // Se o raio começa atrás da origem (t_min < 0.0f)
    if (t_min < 0.0f) {
        if (t_max < 0.0f)
            return -1.0f;  // O objeto está inteiramente atrás da origem do raio.
        
        // O raio começa dentro do OBB. A intersecção que nos interessa é a de saída.
        return t_max;
    }

    // Caso Geral: A intersecção mais próxima e válida (t_min >= 0).
    return t_min;
}


bool colision_with_void(float min_y){
    return min_y <= -30.0f;
}