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


// uniform Catmull–Rom
ClosedCompositeCubicBézierCurve generateClosedBezierCycle(const std::vector<glm::vec4>& points) {
    int n = points.size();
    ClosedCompositeCubicBézierCurve curves;

    for (int i = 0; i < n; i++) {
        int im1 = (i - 1 + n) % n;
        int ip1 = (i + 1) % n;
        int ip2 = (i + 2) % n;

        CubicBézierCurve c;

        c.p1 = points[i];
        c.p2 = points[i] + (points[ip1] - points[im1]) / 6.0f;
        c.p3 = points[ip1] - (points[ip2] - points[i]) / 6.0f;
        c.p4 = points[ip1];

        curves.push_back(c);
    }

    return curves;
}

