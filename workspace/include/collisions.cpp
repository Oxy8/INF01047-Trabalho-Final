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
    bool overlap_x = (aabb1_min.x <= aabb2_max.x) && (aabb1_max .x >= aabb2_min.x);
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



void resolve_collision_obb_aabb(
    glm::vec4& character_pos,
    glm::vec4& character_vel,
    bool& grounded_flag,
    bool update_grounded_flag,
    const OBB& character_obb,
    const glm::vec3& platform_min,
    const glm::vec3& platform_max
)

{
    // 1. Detecção de Colisão
    CollisionResult colision = colision_obb_aabb(character_obb, platform_min, platform_max);
    if (colision.colliding) {
        printf("Colisao detectada!\n");

        float correction_amount = colision.penetration;

        if (correction_amount > 0.0f) {
             glm::vec4 normal_vec = glm::vec4(colision.normal, 0.0f);
             character_pos += normal_vec * correction_amount;
        }

        // Corrige velocidade (Impulso de colisão)
        float vn = glm::dot(glm::vec3(character_vel), colision.normal);

        if (vn < 0.0f) {
             // Anula a componente de velocidade que está penetrando (a normal da colisão que detectamos)
             character_vel -= vn * glm::vec4(colision.normal, 0.0f);
        }
        
        if(update_grounded_flag && colision.normal.y > 0.7f) { // Limiar de 0.7f (aprox. 45 graus)
             // Estabilização vertical: Zera a velocidade y no contato para evitar oscilação
            character_vel.y = 0.0f;

            grounded_flag = true;
        }

    }
    else if (update_grounded_flag) {
        
        float margem = 0.05f; // margem de tolerância
        
        OBB obb_com_margem = character_obb;
        obb_com_margem.center.y -= margem; 

        CollisionResult col_com_margem = colision_obb_aabb(obb_com_margem, platform_min, platform_max);

        if (col_com_margem.colliding && col_com_margem.normal.y > 0.7f) {
            // Estabilização vertical: Zera a velocidade y no contato para evitar oscilação
            character_vel.y = 0.0f;
            
            grounded_flag = true;
        }
    }
/*    
    else {
        // Se não houver colisão, o objeto não está no chão
        if(update_grounded_flag)
            grounded_flag = false;
        
    }
*/
// Teve que ser removido devido à existência de múltiplas plataformas.

}


struct Sphere {
    glm::vec4 center;
    float radius;
};

extern const float raio_colisao_cogumelo = 1.5;
extern const float raio_colisao_bird = 1.5;

bool collision_sphere_sphere(Sphere& s1, Sphere& s2) {
    float threshold = s1.radius + s2.radius;

    return length(s1.center-s2.center) <= threshold;
}


glm::vec4 resolveCollisionSphereAABB(glm::vec4 speed_vec_sphere, Sphere& s1, const glm::vec3& aabb_min, const glm::vec3& aabb_max) {
    
    float closest_x = glm::clamp(s1.center.x, aabb_min.x, aabb_max.x);
    float closest_y = glm::clamp(s1.center.y, aabb_min.y, aabb_max.y);
    float closest_z = glm::clamp(s1.center.z, aabb_min.z, aabb_max.z);

    glm::vec4 closest_point = glm::vec4(closest_x, closest_y, closest_z, 1.0f);

    // deltas para calcular a normal
    float delta_x = s1.center.x - closest_x;
    float delta_y = s1.center.y - closest_y;
    float delta_z = s1.center.z - closest_z;

    glm::vec4 normal = glm::vec4(delta_x, delta_y, delta_z, 0.0f);
    normal = glm::normalize(normal);



    bool collision = glm::length(closest_point-s1.center) <= raio_colisao_cogumelo;


    if (collision) {
        printf("colisao abbb esfera\n");
        return speed_vec_sphere - 2*glm::dot(speed_vec_sphere, normal) * normal;
    }
    else {
        return speed_vec_sphere;
    }
}


bool colision_with_void(float min_y){
    return min_y <= -60.0f;
}