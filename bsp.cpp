#include "bsp.h"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>

using namespace std;

// implementacoes da classe BSPTree
void BSPTree::build(vector<Triangle>& triangles) {
    if (triangles.empty()) {
        return;
    }
    
    // inicia construcao recursiva
    buildTreeRecursive(root.get(), triangles, 0);
}

vector<int> BSPTree::querySegment(const Segment& segment) {
    vector<int> intersected_triangles;
    
    if (root) {
        querySegmentRecursive(root.get(), segment, intersected_triangles);
    }
    
    // remove duplicatas e ordena
    sort(intersected_triangles.begin(), intersected_triangles.end());
    intersected_triangles.erase(
        unique(intersected_triangles.begin(), intersected_triangles.end()),
        intersected_triangles.end()
    );
    
    return intersected_triangles;
}

void BSPTree::splitSpanningTriangle(const Triangle& triangle, const PlaneInfo& plane,
                                   vector<Triangle>& front_parts, vector<Triangle>& back_parts) {
    // classifica vertices em relacao ao plano
    double distances[3];
    int vertex_classification[3]; // -1 = back, 0 = on plane, 1 = front
    
    for (int i = 0; i < 3; i++) {
        distances[i] = plane.classifyPoint(triangle.vertices[i]);
        
        if (distances[i] > EPSILON) {
            vertex_classification[i] = 1; // front
        } else if (distances[i] < -EPSILON) {
            vertex_classification[i] = -1; // back
        } else {
            vertex_classification[i] = 0; // on plane
        }
    }
    
    // conta vertices em cada lado
    int front_count = 0, back_count = 0, on_plane_count = 0;
    for (int i = 0; i < 3; i++) {
        if (vertex_classification[i] == 1) front_count++;
        else if (vertex_classification[i] == -1) back_count++;
        else on_plane_count++;
    }
    
    // casos especiais - triangulo no plano (nao deveria acontecer aqui)
    if (on_plane_count == 3 || (front_count == 0 && back_count == 0)) {
        return; // triangulo coplanar
    }
    
    // caso 1: um vertice na frente, dois atras
    if (front_count == 1 && back_count == 2) {
        splitTriangleOneTwo(triangle, plane, vertex_classification, true, front_parts, back_parts);
    }
    // caso 2: dois vertices na frente, um atras  
    else if (front_count == 2 && back_count == 1) {
        splitTriangleOneTwo(triangle, plane, vertex_classification, false, front_parts, back_parts);
    }
    // casos com vertices no plano
    else if (on_plane_count > 0) {
        splitTriangleWithVertexOnPlane(triangle, plane, vertex_classification, front_parts, back_parts);
    }
}

void BSPTree::splitTriangleOneTwo(const Triangle& triangle, const PlaneInfo& plane,
                                 const int vertex_classification[3], bool one_front_two_back,
                                 vector<Triangle>& front_parts, vector<Triangle>& back_parts) {
    
    // encontra o vertice isolado e os dois do outro lado
    int isolated_vertex = -1;
    int other_vertices[2];
    int other_count = 0;
    
    int target_class = one_front_two_back ? 1 : -1; // procura o lado com um vertice
    
    for (int i = 0; i < 3; i++) {
        if (vertex_classification[i] == target_class) {
            isolated_vertex = i;
        } else if (vertex_classification[i] == -target_class) {
            other_vertices[other_count++] = i;
        }
    }
    
    if (isolated_vertex == -1 || other_count != 2) {
        return; // erro na classificacao
    }
    
    // calcula pontos de intersecao nas duas arestas que conectam o vertice isolado aos outros
    Point3D intersection1, intersection2;
    bool found1 = calculateEdgePlaneIntersection(triangle.vertices[isolated_vertex], 
                                                triangle.vertices[other_vertices[0]], 
                                                plane, intersection1);
    bool found2 = calculateEdgePlaneIntersection(triangle.vertices[isolated_vertex], 
                                                triangle.vertices[other_vertices[1]], 
                                                plane, intersection2);
    
    if (!found1 || !found2) {
        return; // nao conseguiu calcular intersecoes
    }
    
    // cria triangulos
    if (one_front_two_back) {
        // um triangulo na frente (vertice isolado + duas intersecoes)
        Triangle front_tri(triangle.vertices[isolated_vertex], intersection1, intersection2, triangle.id);
        front_parts.push_back(front_tri);
        
        // dois triangulos atras
        Triangle back_tri1(triangle.vertices[other_vertices[0]], triangle.vertices[other_vertices[1]], 
                          intersection1, triangle.id);
        Triangle back_tri2(triangle.vertices[other_vertices[1]], intersection2, intersection1, triangle.id);
        back_parts.push_back(back_tri1);
        back_parts.push_back(back_tri2);
    } else {
        // dois triangulos na frente
        Triangle front_tri1(triangle.vertices[other_vertices[0]], triangle.vertices[other_vertices[1]], 
                           intersection1, triangle.id);
        Triangle front_tri2(triangle.vertices[other_vertices[1]], intersection2, intersection1, triangle.id);
        front_parts.push_back(front_tri1);
        front_parts.push_back(front_tri2);
        
        // um triangulo atras
        Triangle back_tri(triangle.vertices[isolated_vertex], intersection1, intersection2, triangle.id);
        back_parts.push_back(back_tri);
    }
}

void BSPTree::splitTriangleWithVertexOnPlane(const Triangle& triangle, const PlaneInfo& plane,
                                           const int vertex_classification[3],
                                           vector<Triangle>& front_parts, vector<Triangle>& back_parts) {
    
    // encontra vertices em cada categoria
    vector<int> front_vertices, back_vertices, plane_vertices;
    
    for (int i = 0; i < 3; i++) {
        if (vertex_classification[i] == 1) {
            front_vertices.push_back(i);
        } else if (vertex_classification[i] == -1) {
            back_vertices.push_back(i);
        } else {
            plane_vertices.push_back(i);
        }
    }
    
    // caso: um vertice no plano, um na frente, um atras
    if (plane_vertices.size() == 1 && front_vertices.size() == 1 && back_vertices.size() == 1) {
        int plane_v = plane_vertices[0];
        int front_v = front_vertices[0];
        int back_v = back_vertices[0];
        
        // calcula intersecao na aresta entre front e back
        Point3D intersection;
        if (calculateEdgePlaneIntersection(triangle.vertices[front_v], triangle.vertices[back_v], 
                                          plane, intersection)) {
            // triangulo na frente: front_vertex + plane_vertex + intersection
            Triangle front_tri(triangle.vertices[front_v], triangle.vertices[plane_v], intersection, triangle.id);
            front_parts.push_back(front_tri);
            
            // triangulo atras: back_vertex + plane_vertex + intersection  
            Triangle back_tri(triangle.vertices[back_v], triangle.vertices[plane_v], intersection, triangle.id);
            back_parts.push_back(back_tri);
        }
    }
    // caso: dois vertices no plano, um de um lado
    else if (plane_vertices.size() == 2) {
        if (front_vertices.size() == 1) {
            // triangulo inteiro vai para frente
            front_parts.push_back(triangle);
        } else if (back_vertices.size() == 1) {
            // triangulo inteiro vai para tras
            back_parts.push_back(triangle);
        }
    }
}

bool BSPTree::calculateEdgePlaneIntersection(const Point3D& p1, const Point3D& p2, 
                                           const PlaneInfo& plane, Point3D& intersection) {
    Point3D direction = p2 - p1;
    double denominator = plane.a * direction.x + plane.b * direction.y + plane.c * direction.z;
    
    if (fabs(denominator) < EPSILON) {
        return false; // aresta paralela ao plano
    }
    
    double numerator = -(plane.a * p1.x + plane.b * p1.y + plane.c * p1.z + plane.d);
    double t = numerator / denominator;
    
    if (t < 0 || t > 1) {
        return false; // intersecao fora do segmento
    }
    
    intersection = p1 + direction * t;
    return true;
}

void BSPTree::buildTreeRecursive(BSPNode* node, vector<Triangle>& triangles, int depth) {
    // criterio de parada: poucos triangulos ou profundidade maxima
    const int MAX_TRIANGLES_PER_LEAF = 10;
    const int MAX_DEPTH = 20;
    
    if (triangles.size() <= MAX_TRIANGLES_PER_LEAF || depth >= MAX_DEPTH) {
        // cria folha
        node->is_leaf = true;
        node->triangles = triangles;
        return;
    }
    
    // escolhe plano de divisao
    PlaneInfo splitting_plane = chooseSplittingPlane(triangles);
    
    // se nao conseguiu encontrar plano util, cria folha
    if (splitting_plane.a == 0 && splitting_plane.b == 0 && 
        splitting_plane.c == 0 && splitting_plane.d == 0) {
        node->is_leaf = true;
        node->triangles = triangles;
        return;
    }
    
    // divide triangulos
    vector<Triangle> front_triangles, back_triangles, coplanar_triangles;
    splitTriangles(triangles, splitting_plane, front_triangles, back_triangles, coplanar_triangles);
    
    // configura no interno
    node->makeInternal(splitting_plane);
    
    // adiciona triangulos coplanares ao no atual
    node->triangles = coplanar_triangles;
    
    // constroi filhos recursivamente
    if (!front_triangles.empty()) {
        buildTreeRecursive(node->front.get(), front_triangles, depth + 1);
    }
    
    if (!back_triangles.empty()) {
        buildTreeRecursive(node->back.get(), back_triangles, depth + 1);
    }
}

PlaneInfo BSPTree::chooseSplittingPlane(const vector<Triangle>& triangles) {
    // estrategia simples: usa o plano do primeiro triangulo
    if (triangles.empty()) {
        return PlaneInfo();
    }
    
    const Triangle& first_triangle = triangles[0];
    PlaneInfo plane;
    first_triangle.getPlaneEquation(plane.a, plane.b, plane.c, plane.d);
    
    return plane;
}

void BSPTree::splitTriangles(const vector<Triangle>& triangles, const PlaneInfo& plane,
                            vector<Triangle>& front_triangles, vector<Triangle>& back_triangles,
                            vector<Triangle>& coplanar_triangles) {
    
    for (const Triangle& triangle : triangles) {
        PlaneInfo::TriangleClassification classification = plane.classifyTriangle(triangle);
        
        switch (classification) {
            case PlaneInfo::FRONT:
                front_triangles.push_back(triangle);
                break;
                
            case PlaneInfo::BACK:
                back_triangles.push_back(triangle);
                break;
                
            case PlaneInfo::COPLANAR:
                coplanar_triangles.push_back(triangle);
                break;
                
            case PlaneInfo::SPANNING:
                // divide o triangulo que cruza o plano
                vector<Triangle> front_parts, back_parts;
                splitSpanningTriangle(triangle, plane, front_parts, back_parts);
                
                // adiciona as partes aos vetores correspondentes
                front_triangles.insert(front_triangles.end(), front_parts.begin(), front_parts.end());
                back_triangles.insert(back_triangles.end(), back_parts.begin(), back_parts.end());
                break;
        }
    }
}

void BSPTree::querySegmentRecursive(BSPNode* node, const Segment& segment, 
                                   vector<int>& intersected_triangles) {
    if (!node) {
        return;
    }
    
    // verifica intersecoes com triangulos do no atual
    for (const Triangle& triangle : node->triangles) {
        if (segmentIntersectsTriangle(segment, triangle)) {
            intersected_triangles.push_back(triangle.id);
        }
    }
    
    // se eh folha, termina
    if (node->is_leaf) {
        return;
    }
    
    // classifica pontos do segmento em relacao ao plano
    double start_dist = node->plane.classifyPoint(segment.start);
    double end_dist = node->plane.classifyPoint(segment.end);
    
    // se segmento esta completamente na frente
    if (start_dist > EPSILON && end_dist > EPSILON) {
        querySegmentRecursive(node->front.get(), segment, intersected_triangles);
    }
    // se segmento esta completamente atras
    else if (start_dist < -EPSILON && end_dist < -EPSILON) {
        querySegmentRecursive(node->back.get(), segment, intersected_triangles);
    }
    // se segmento cruza o plano ou esta no plano
    else {
        querySegmentRecursive(node->front.get(), segment, intersected_triangles);
        querySegmentRecursive(node->back.get(), segment, intersected_triangles);
    }
}

bool BSPTree::segmentIntersectsTriangle(const Segment& segment, const Triangle& triangle) {
    // calcula direcao do segmento
    Point3D direction = segment.getDirection();
    double segment_length = direction.length();
    
    if (segment_length < EPSILON) {
        return false;  // segmento degenerado
    }
    
    direction = direction * (1.0 / segment_length);  // normaliza
    
    // testa intersecao raio-triangulo
    double t;
    if (rayIntersectsTriangle(segment.start, direction, triangle, t)) {
        // verifica se intersecao esta dentro do segmento
        return (t >= 0 && t <= segment_length);
    }
    
    return false;
}

bool BSPTree::rayIntersectsTriangle(const Point3D& ray_origin, const Point3D& ray_direction,
                                   const Triangle& triangle, double& t) {
    // algoritmo de moller-trumbore para intersecao raio-triangulo
    const Point3D& v0 = triangle.vertices[0];
    const Point3D& v1 = triangle.vertices[1];
    const Point3D& v2 = triangle.vertices[2];
    
    Point3D edge1 = v1 - v0;
    Point3D edge2 = v2 - v0;
    
    Point3D h = ray_direction.cross(edge2);
    double a = edge1.dot(h);
    
    if (fabs(a) < EPSILON) {
        return false;  // raio paralelo ao triangulo
    }
    
    double f = 1.0 / a;
    Point3D s = ray_origin - v0;
    double u = f * s.dot(h);
    
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    
    Point3D q = s.cross(edge1);
    double v = f * ray_direction.dot(q);
    
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    
    t = f * edge2.dot(q);
    
    return (t > EPSILON);
}

int BSPTree::getTriangleCount() const {
    // TODO: implementar contagem recursiva
    return 0;
}

int BSPTree::getNodeCount() const {
    // TODO: implementar contagem recursiva
    return 0;
}

int BSPTree::getMaxDepth() const {
    // TODO: implementar calculo recursivo
    return 0;
}

void BSPTree::printTreeInfo() const {
    #ifdef DEBUG
    cout << "=== informacoes da arvore bsp ===" << endl;
    cout << "triangulos: " << getTriangleCount() << endl;
    cout << "nos: " << getNodeCount() << endl;
    cout << "profundidade maxima: " << getMaxDepth() << endl;
    cout << "================================" << endl;
    #endif
}

// implementacoes das funcoes utilitarias geometricas
namespace GeometryUtils {
    
    double distance(const Point3D& a, const Point3D& b) {
        Point3D diff = b - a;
        return diff.length();
    }
    
    bool pointsEqual(const Point3D& a, const Point3D& b, double tolerance) {
        return distance(a, b) < tolerance;
    }
    
    Point3D getTriangleCentroid(const Triangle& triangle) {
        return (triangle.vertices[0] + triangle.vertices[1] + triangle.vertices[2]) * (1.0 / 3.0);
    }
    
    double getTriangleArea(const Triangle& triangle) {
        Point3D edge1 = triangle.vertices[1] - triangle.vertices[0];
        Point3D edge2 = triangle.vertices[2] - triangle.vertices[0];
        return edge1.cross(edge2).length() * 0.5;
    }
    
    bool isPointInTriangle(const Point3D& point, const Triangle& triangle) {
        // verifica se ponto esta no plano do triangulo
        if (!triangle.isPointInPlane(point)) {
            return false;
        }
        
        // usa coordenadas baricentricas para verificar se esta dentro
        Point3D v0 = triangle.vertices[2] - triangle.vertices[0];
        Point3D v1 = triangle.vertices[1] - triangle.vertices[0];
        Point3D v2 = point - triangle.vertices[0];
        
        double dot00 = v0.dot(v0);
        double dot01 = v0.dot(v1);
        double dot02 = v0.dot(v2);
        double dot11 = v1.dot(v1);
        double dot12 = v1.dot(v2);
        
        double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
        
        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }
    
    bool rayPlaneIntersection(const Point3D& ray_origin, const Point3D& ray_direction,
                             const PlaneInfo& plane, Point3D& intersection_point) {
        double denominator = plane.a * ray_direction.x + 
                           plane.b * ray_direction.y + 
                           plane.c * ray_direction.z;
        
        if (fabs(denominator) < EPSILON) {
            return false;  // raio paralelo ao plano
        }
        
        double numerator = -(plane.a * ray_origin.x + 
                           plane.b * ray_origin.y + 
                           plane.c * ray_origin.z + 
                           plane.d);
        
        double t = numerator / denominator;
        
        if (t < 0) {
            return false;  // intersecao atras da origem do raio
        }
        
        intersection_point = ray_origin + ray_direction * t;
        return true;
    }
    
    bool isPointOnSegment(const Point3D& point, const Segment& segment, double tolerance) {
        Point3D segment_vec = segment.end - segment.start;
        Point3D point_vec = point - segment.start;
        
        double segment_length = segment_vec.length();
        if (segment_length < tolerance) {
            return pointsEqual(point, segment.start, tolerance);
        }
        
        double projection = point_vec.dot(segment_vec) / (segment_length * segment_length);
        
        if (projection < 0 || projection > 1) {
            return false;
        }
        
        Point3D closest_point = segment.start + segment_vec * projection;
        return pointsEqual(point, closest_point, tolerance);
    }
} 