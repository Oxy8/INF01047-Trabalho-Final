#include <cmath>
#include <cstdio>
#include <cstdlib>

// Headers abaixo são específicos de C++
#include <set>
#include <map>
#include <stack>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

// Headers das bibliotecas OpenGL
#include <glad/glad.h>   // Criação de contexto OpenGL 3.3
#include <GLFW/glfw3.h>  // Criação de janelas do sistema operacional

// Headers da biblioteca GLM: criação de matrizes e vetores.
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>

// Headers da biblioteca para carregar modelos obj
#include <tiny_obj_loader.h>

#include <stb_image.h>

// Headers locais, definidos na pasta "include/"
#include "utils.h"
#include "matrices.h"

extern std::vector<std::vector<glm::vec4>> passaros = {
    {
    glm::vec4(6.0f + 2.5f * cos(0.0f),                 2.0f, 3.5f + 2.5f * sin(0.0f),        1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(45.0f)),  2.5f, 3.5f + 2.5f * sin(glm::radians(45.0f)), 1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(90.0f)),  2.8f, 3.5f + 2.5f * sin(glm::radians(90.0f)), 1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(135.0f)), 3.0f, 3.5f + 2.5f * sin(glm::radians(135.0f)), 1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(180.0f)), 2.7f, 3.5f + 2.5f * sin(glm::radians(180.0f)), 1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(225.0f)), 2.4f, 3.5f + 2.5f * sin(glm::radians(225.0f)), 1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(270.0f)), 2.0f, 3.5f + 2.5f * sin(glm::radians(270.0f)), 1.0f),
    glm::vec4(6.0f + 2.5f * cos(glm::radians(315.0f)), 1.9f, 3.5f + 2.5f * sin(glm::radians(315.0f)), 1.0f)
    },
    {
    glm::vec4(-3.6f, 2.5f, -6.0f, 1.0f),
    glm::vec4(0.0f, 2.5f, -7.0f, 1.0f),
    glm::vec4(3.6f, 2.5f, -6.0f, 1.0f),
    glm::vec4(4.2f, 2.5f, 0.0f, 1.0f),
    glm::vec4(3.6f, 2.5f, 6.0f, 1.0f),
    glm::vec4(0.0f, 2.5f, 7.0f, 1.0f),
    glm::vec4(-3.6f, 2.5f, 6.0f, 1.0f),
    glm::vec4(-4.2f, 2.5f, 0.0f, 1.0f),
    }
};
extern int n_passaros = passaros.size();


struct CubicBézierCurve {
    glm::vec4 p1;
    glm::vec4 p2;
    glm::vec4 p3;
    glm::vec4 p4;

    // const
    glm::vec4 point(float t) const {
        
        glm::vec4 c12  = p1 + t*(p2 - p1);
        glm::vec4 c23  = p2 + t*(p3 - p2);
        glm::vec4 c34  = p3 + t*(p4 - p3);

        glm::vec4 c123 = c12 + t*(c23 - c12);
        glm::vec4 c234 = c23 + t*(c34 - c23);

        glm::vec4 p = c123 + t*(c234 - c123);
        return p;
    }

    glm::vec4 derivative(float t) const {
        glm::vec4 q1 = 3.0f*(p2 - p1);
        glm::vec4 q2 = 3.0f*(p3 - p2);
        glm::vec4 q3 = 3.0f*(p4 - p3);

        glm::vec4 d12 = q1 + t*(q2 - q1);
        glm::vec4 d23 = q2 + t*(q3 - q2);

        glm::vec4 d123 = d12 + t*(d23 - d12);
        
        return d123;
    }
};

typedef std::vector<CubicBézierCurve> ClosedCompositeCubicBézierCurve;

glm::mat4 prepareDrawBird(ClosedCompositeCubicBézierCurve curve, float time) {
    
    int n_segments = curve.size();
    if (n_segments == 0) { return Matrix_Identity(); };

    float remainder = std::fmod(time, n_segments);
    int segment_index = floor(remainder);

    CubicBézierCurve selected_segment = curve[segment_index];

    glm::vec4 draw_point = selected_segment.point(remainder-(float)segment_index);

    glm::vec4 tangent = selected_segment.derivative(remainder-(float)segment_index);
    tangent = normalize(tangent);

    float yaw = atan2(tangent.x,tangent.z);
    float pitch = -asin(tangent.y);

    glm::mat4 model = Matrix_Identity()
                    * Matrix_Translate(draw_point.x, draw_point.y, draw_point.z)
                    * Matrix_Rotate_Y(yaw)
                    * Matrix_Rotate_X(pitch);
    return model;
}


// uniform Catmull–Rom modified
ClosedCompositeCubicBézierCurve generateClosedBezierCycle(const std::vector<glm::vec4>& points) {
    
    int n = points.size();
    
    ClosedCompositeCubicBézierCurve curves;

    for (int i = 0; i < n; i++) {
        int prev = (i - 1 + n) % n;
        int next = (i + 1) % n;
        int next_next = (i + 2) % n;

        CubicBézierCurve c;

        c.p1 = points[i];
        c.p2 = points[i] + (points[next] - points[prev]) / 4.0f;
        c.p3 = points[next] - (points[next_next] - points[i]) / 4.0f;
        c.p4 = points[next];

        curves.push_back(c);
    }

    return curves;
}





GLuint LoadCubemap(const std::vector<std::string>& faces)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    stbi_set_flip_vertically_on_load(false);

    int width, height, nrChannels;
    for (GLuint i = 0; i < faces.size(); i++)
    {
        unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
            GLenum format = nrChannels == 3 ? GL_RGB : GL_RGBA;
            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data
            );
            stbi_image_free(data);
        }
        else
        {
            printf("Failed to load cubemap face: %s\n", faces[i].c_str());
            stbi_image_free(data);
        }
    }

    // Parâmetros padrão para skybox
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

    stbi_set_flip_vertically_on_load(true);

    return textureID;
}

