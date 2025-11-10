// AABB = Axis Aligned Bounding Box

const colision_aabb_plane(glm::vec3& aabb_min, const glm::vec3& aabb_max,
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



