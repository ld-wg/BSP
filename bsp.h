#ifndef BSP_H
#define BSP_H

#include <vector>
#include <memory>
#include <cmath>

// tolerancia numerica para comparacoes geometricas
const double EPSILON = 1e-9;

// estrutura para representar um ponto 3d
struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
    
    // operadores basicos
    Point3D operator+(const Point3D& other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }
    
    Point3D operator-(const Point3D& other) const {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }
    
    Point3D operator*(double scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }
    
    // produto escalar
    double dot(const Point3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    // produto vetorial
    Point3D cross(const Point3D& other) const {
        return Point3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    // magnitude do vetor
    double length() const {
        return sqrt(x * x + y * y + z * z);
    }
    
    // normalizacao
    Point3D normalize() const {
        double len = length();
        if (len < EPSILON) return Point3D(0, 0, 0);
        return Point3D(x / len, y / len, z / len);
    }
};

// estrutura para representar um triangulo
struct Triangle {
    Point3D vertices[3];  // vertices a, b, c
    int id;              // identificador do triangulo (1-based como no problema)
    
    Triangle() : id(0) {}
    Triangle(const Point3D& a, const Point3D& b, const Point3D& c, int triangle_id)
        : id(triangle_id) {
        vertices[0] = a;
        vertices[1] = b;
        vertices[2] = c;
    }
    
    // calcula o vetor normal do triangulo
    Point3D getNormal() const {
        Point3D edge1 = vertices[1] - vertices[0];
        Point3D edge2 = vertices[2] - vertices[0];
        return edge1.cross(edge2).normalize();
    }
    
    // calcula a equacao do plano: ax + by + cz + d = 0
    void getPlaneEquation(double& a, double& b, double& c, double& d) const {
        Point3D normal = getNormal();
        a = normal.x;
        b = normal.y;
        c = normal.z;
        d = -(normal.dot(vertices[0]));
    }
    
    // verifica se um ponto esta no plano do triangulo
    bool isPointInPlane(const Point3D& point) const {
        double a, b, c, d;
        getPlaneEquation(a, b, c, d);
        return fabs(a * point.x + b * point.y + c * point.z + d) < EPSILON;
    }
};

// estrutura para representar um segmento de reta
struct Segment {
    Point3D start;
    Point3D end;
    
    Segment() {}
    Segment(const Point3D& s, const Point3D& e) : start(s), end(e) {}
    
    // calcula a direcao do segmento
    Point3D getDirection() const {
        return end - start;
    }
    
    // calcula o comprimento do segmento
    double getLength() const {
        return getDirection().length();
    }
};

// estrutura para representar um plano de divisao na bsp
struct PlaneInfo {
    double a, b, c, d;  // equacao do plano: ax + by + cz + d = 0
    
    PlaneInfo() : a(0), b(0), c(0), d(0) {}
    PlaneInfo(double a, double b, double c, double d) : a(a), b(b), c(c), d(d) {}
    
    // classifica um ponto em relacao ao plano
    // retorna: positivo se na frente, negativo se atras, ~0 se no plano
    double classifyPoint(const Point3D& point) const {
        return a * point.x + b * point.y + c * point.z + d;
    }
    
    // classifica um triangulo em relacao ao plano
    enum TriangleClassification {
        FRONT,      // todo o triangulo esta na frente do plano
        BACK,       // todo o triangulo esta atras do plano
        COPLANAR,   // triangulo esta no plano
        SPANNING    // triangulo cruza o plano
    };
    
    TriangleClassification classifyTriangle(const Triangle& triangle) const {
        double distances[3];
        int front_count = 0, back_count = 0, coplanar_count = 0;
        
        for (int i = 0; i < 3; i++) {
            distances[i] = classifyPoint(triangle.vertices[i]);
            
            if (distances[i] > EPSILON) {
                front_count++;
            } else if (distances[i] < -EPSILON) {
                back_count++;
            } else {
                coplanar_count++;
            }
        }
        
        if (front_count == 3) return FRONT;
        if (back_count == 3) return BACK;
        if (coplanar_count == 3) return COPLANAR;
        return SPANNING;
    }
};

// no da arvore bsp
class BSPNode {
public:
    PlaneInfo plane;                        // plano divisor (apenas para nos internos)
    std::vector<Triangle> triangles;        // triangulos armazenados neste no
    std::unique_ptr<BSPNode> front;         // filho da frente
    std::unique_ptr<BSPNode> back;          // filho de tras
    bool is_leaf;                           // indica se e um no folha
    
    BSPNode() : is_leaf(true) {}
    
    // construtor para no interno
    BSPNode(const PlaneInfo& splitting_plane) 
        : plane(splitting_plane), is_leaf(false) {}
    
    // adiciona um triangulo ao no (apenas para folhas)
    void addTriangle(const Triangle& triangle) {
        if (is_leaf) {
            triangles.push_back(triangle);
        }
    }
    
    // converte para no interno com plano divisor
    void makeInternal(const PlaneInfo& splitting_plane) {
        plane = splitting_plane;
        is_leaf = false;
        front.reset(new BSPNode());
        back.reset(new BSPNode());
    }
    
    // verifica se o no tem filhos
    bool hasChildren() const {
        return !is_leaf && front && back;
    }
};

// classe principal da arvore bsp
class BSPTree {
private:
    std::unique_ptr<BSPNode> root;
    
    // metodos auxiliares para construcao recursiva
    void buildTreeRecursive(BSPNode* node, std::vector<Triangle>& triangles, int depth = 0);
    PlaneInfo chooseSplittingPlane(const std::vector<Triangle>& triangles);
    void splitTriangles(const std::vector<Triangle>& triangles, const PlaneInfo& plane,
                       std::vector<Triangle>& front_triangles, std::vector<Triangle>& back_triangles,
                       std::vector<Triangle>& coplanar_triangles);
    void splitSpanningTriangle(const Triangle& triangle, const PlaneInfo& plane,
                              std::vector<Triangle>& front_parts, std::vector<Triangle>& back_parts);
    void splitTriangleOneTwo(const Triangle& triangle, const PlaneInfo& plane,
                            const int vertex_classification[3], bool one_front_two_back,
                            std::vector<Triangle>& front_parts, std::vector<Triangle>& back_parts);
    void splitTriangleWithVertexOnPlane(const Triangle& triangle, const PlaneInfo& plane,
                                       const int vertex_classification[3],
                                       std::vector<Triangle>& front_parts, std::vector<Triangle>& back_parts);
    bool calculateEdgePlaneIntersection(const Point3D& p1, const Point3D& p2, 
                                       const PlaneInfo& plane, Point3D& intersection);
    
    // metodos auxiliares para consulta
    void querySegmentRecursive(BSPNode* node, const Segment& segment, 
                              std::vector<int>& intersected_triangles);
    bool segmentIntersectsTriangle(const Segment& segment, const Triangle& triangle);
    bool rayIntersectsTriangle(const Point3D& ray_origin, const Point3D& ray_direction,
                              const Triangle& triangle, double& t);

public:
    BSPTree() : root(new BSPNode()) {}
    
    // constroi a arvore bsp a partir de uma lista de triangulos
    void build(std::vector<Triangle>& triangles);
    
    // consulta quais triangulos sao intersectados por um segmento
    std::vector<int> querySegment(const Segment& segment);
    
    // metodos de utilidade
    int getTriangleCount() const;
    int getNodeCount() const;
    int getMaxDepth() const;
    
    // debug - imprime informacoes da arvore
    void printTreeInfo() const;
};

// estruturas BSPInput e BSPOutput foram movidas para io.h

// funcoes utilitarias geometricas
namespace GeometryUtils {
    // calcula a distancia entre dois pontos
    double distance(const Point3D& a, const Point3D& b);
    
    // verifica se dois pontos sao aproximadamente iguais
    bool pointsEqual(const Point3D& a, const Point3D& b, double tolerance = EPSILON);
    
    // calcula o centroide de um triangulo
    Point3D getTriangleCentroid(const Triangle& triangle);
    
    // calcula a area de um triangulo
    double getTriangleArea(const Triangle& triangle);
    
    // verifica se um ponto esta dentro de um triangulo (assumindo coplanaridade)
    bool isPointInTriangle(const Point3D& point, const Triangle& triangle);
    
    // calcula a intersecao entre uma reta e um plano
    bool rayPlaneIntersection(const Point3D& ray_origin, const Point3D& ray_direction,
                             const PlaneInfo& plane, Point3D& intersection_point);
    
    // verifica se um ponto esta dentro de um segmento
    bool isPointOnSegment(const Point3D& point, const Segment& segment, double tolerance = EPSILON);
}

// configuracoes da bsp tree
struct BSPConfig {
    int max_triangles_per_leaf;  // maximo de triangulos por folha
    int max_depth;               // profundidade maxima da arvore
    double splitting_tolerance;  // tolerancia para divisao de triangulos
    
    BSPConfig() : max_triangles_per_leaf(10), max_depth(20), splitting_tolerance(EPSILON) {}
};

#endif // BSP_H 