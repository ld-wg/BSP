#include "io.h"
#include <iostream>
#include <sstream>
#include <algorithm>

using namespace std;

// implementacoes da struct BSPInput
void BSPInput::clear() {
    points.clear();
    triangles.clear();
    segments.clear();
}

int BSPInput::getPointCount() const {
    return static_cast<int>(points.size()) - 1;  // -1 para descontar ponto dummy
}

// implementacoes da struct BSPOutput
void BSPOutput::clear() {
    results.clear();
}

void BSPOutput::addResult(const vector<int>& triangle_ids) {
    results.push_back(triangle_ids);
}

int BSPOutput::getQueryCount() const {
    return static_cast<int>(results.size());
}

// implementacoes das funcoes de entrada
namespace InputFunctions {
    
    bool readInput(BSPInput& input) {
        int n, t, l;
        
        // le cabecalho
        if (!readHeader(n, t, l)) {
            return false;
        }
        
        // prepara estruturas
        input.clear();
        input.points.reserve(n + 1);
        input.triangles.reserve(t);
        input.segments.reserve(l);
        
        // le dados
        return readPoints(input, n) && 
               readTriangles(input, t, n) && 
               readSegments(input, l);
    }
    
    bool validateInput(const BSPInput& input) {
        // verifica se temos pelo menos um triangulo
        if (input.triangles.empty()) {
            cerr << "erro: nenhum triangulo encontrado" << endl;
            return false;
        }
        
        // verifica se temos pelo menos um segmento
        if (input.segments.empty()) {
            cerr << "erro: nenhum segmento encontrado" << endl;
            return false;
        }
        
        // verifica se os pontos estao dentro do range esperado (1-99)
        for (size_t i = 1; i < input.points.size(); i++) {
            const Point3D& p = input.points[i];
            if (p.x < 1 || p.x > 99 || p.y < 1 || p.y > 99 || p.z < 1 || p.z > 99) {
                cerr << "erro: coordenadas do ponto " << i << " fora do range (1-99)" << endl;
                return false;
            }
        }
        
        return true;
    }
    
    bool readHeader(int& n, int& t, int& l) {
        if (!(cin >> n >> t >> l)) {
            cerr << "erro: falha ao ler parametros iniciais (n t l)" << endl;
            return false;
        }
        
        if (n <= 0 || t <= 0 || l <= 0) {
            cerr << "erro: parametros devem ser positivos" << endl;
            return false;
        }
        
        return true;
    }
    
    bool readPoints(BSPInput& input, int n) {
        // adiciona ponto dummy no indice 0 para indexacao 1-based
        input.points.push_back(Point3D(0, 0, 0));
        
        // le coordenadas dos pontos
        for (int i = 1; i <= n; i++) {
            double x, y, z;
            if (!(cin >> x >> y >> z)) {
                cerr << "erro: falha ao ler coordenadas do ponto " << i << endl;
                return false;
            }
            input.points.push_back(Point3D(x, y, z));
        }
        
        return true;
    }
    
    bool readTriangles(BSPInput& input, int t, int n) {
        // le indices dos vertices dos triangulos
        for (int i = 1; i <= t; i++) {
            int v1, v2, v3;
            if (!(cin >> v1 >> v2 >> v3)) {
                cerr << "erro: falha ao ler vertices do triangulo " << i << endl;
                return false;
            }
            
            // verifica se os indices sao validos
            if (v1 < 1 || v1 > n || v2 < 1 || v2 > n || v3 < 1 || v3 > n) {
                cerr << "erro: indices de vertices invalidos no triangulo " << i << endl;
                return false;
            }
            
            // cria triangulo usando os pontos correspondentes
            Triangle triangle(input.points[v1], input.points[v2], input.points[v3], i);
            input.triangles.push_back(triangle);
        }
        
        return true;
    }
    
    bool readSegments(BSPInput& input, int l) {
        // le coordenadas dos segmentos
        for (int i = 0; i < l; i++) {
            double xa, ya, za, xb, yb, zb;
            if (!(cin >> xa >> ya >> za >> xb >> yb >> zb)) {
                cerr << "erro: falha ao ler coordenadas do segmento " << (i + 1) << endl;
                return false;
            }
            
            Point3D start(xa, ya, za);
            Point3D end(xb, yb, zb);
            input.segments.push_back(Segment(start, end));
        }
        
        return true;
    }
}

// implementacoes das funcoes de saida
namespace OutputFunctions {
    
    void writeOutput(const BSPOutput& output) {
        for (const auto& result : output.results) {
            writeResultLine(result);
        }
    }
    
    void writeResultLine(const vector<int>& triangle_ids) {
        // escreve numero de triangulos intersectados
        cout << triangle_ids.size();
        
        // escreve indices dos triangulos intersectados
        for (int triangle_id : triangle_ids) {
            cout << " " << triangle_id;
        }
        
        cout << endl;
    }
    
    string formatResult(const vector<int>& triangle_ids) {
        ostringstream oss;
        oss << triangle_ids.size();
        
        for (int triangle_id : triangle_ids) {
            oss << " " << triangle_id;
        }
        
        return oss.str();
    }
}

// implementacoes das funcoes de debug
namespace DebugFunctions {
    
    void printDebugInfo(const BSPInput& input, const BSPTree& tree) {
        cout << "=== informacoes de debug ===" << endl;
        printInputStats(input);
        printTreeStats(tree);
        cout << "============================" << endl;
    }
    
    void printInputStats(const BSPInput& input) {
        cout << "pontos: " << input.getPointCount() << endl;
        cout << "triangulos: " << input.triangles.size() << endl;
        cout << "segmentos: " << input.segments.size() << endl;
    }
    
    void printTreeStats(const BSPTree& tree) {
        // TODO: implementar quando os metodos da BSPTree estiverem prontos
        // cout << "nos da arvore: " << tree.getNodeCount() << endl;
        // cout << "profundidade maxima: " << tree.getMaxDepth() << endl;
        cout << "estatisticas da arvore: em implementacao" << endl;
        // suprime warning de parametro nao utilizado
        (void)tree;
    }
}

// implementacoes das funcoes de tratamento de erros
namespace ErrorHandling {
    
    void handleError(const string& message) {
        handleError(message, 1);
    }
    
    void handleError(const string& message, int exit_code) {
        cerr << "erro: " << message << endl;
        exit(exit_code);
    }
    
    bool checkInputState() {
        if (cin.fail()) {
            cerr << "erro: estado de entrada invalido" << endl;
            return false;
        }
        return true;
    }
} 