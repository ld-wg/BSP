#include <iostream>
#include <vector>
#include <algorithm>
#include <cstring>
#include "bsp.h"
#include "io.h"

using namespace std;

// flag global para debug
bool DEBUG_MODE = false;

// funcao para construir a arvore bsp
void buildBSPTree(BSPTree& tree, vector<Triangle>& triangles) {
    if (DEBUG_MODE) {
        cout << "construindo arvore bsp com " << triangles.size() << " triangulos..." << endl;
    }
    
    tree.build(triangles);
    
    if (DEBUG_MODE) {
        cout << "arvore bsp construida com sucesso!" << endl;
    }
}

// funcao para processar consultas dos segmentos
void processSegmentQueries(BSPTree& tree, const vector<Segment>& segments, BSPOutput& output) {
    if (DEBUG_MODE) {
        cout << "processando " << segments.size() << " consultas de segmentos..." << endl;
    }
    
    output.results.reserve(segments.size());
    
    for (size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments[i];
        
        // consulta quais triangulos sao intersectados pelo segmento
        vector<int> intersected_triangles = tree.querySegment(segment);
        
        // ordena os indices dos triangulos intersectados
        sort(intersected_triangles.begin(), intersected_triangles.end());
        
        // adiciona o resultado
        output.addResult(intersected_triangles);
    }
    
    if (DEBUG_MODE) {
        cout << "consultas processadas com sucesso!" << endl;
    }
}

// ponto de entrada principal do programa
int main(int argc, char* argv[]) {
    // verifica se a flag de debug foi passada
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--debug") == 0 || strcmp(argv[i], "-d") == 0) {
            DEBUG_MODE = true;
            break;
        }
    }
    
    try {
        // estruturas de dados principais
        BSPInput input;
        BSPTree tree;
        BSPOutput output;
        
        // etapa 1: leitura dos dados de entrada
        if (!InputFunctions::readInput(input)) {
            ErrorHandling::handleError("falha na leitura dos dados de entrada");
        }
        
        // etapa 2: validacao dos dados
        if (!InputFunctions::validateInput(input)) {
            ErrorHandling::handleError("dados de entrada invalidos");
        }
        
        // etapa 3: construcao da arvore bsp
        buildBSPTree(tree, input.triangles);
        
        // etapa 4: processamento das consultas
        processSegmentQueries(tree, input.segments, output);
        
        // etapa 5: escrita dos resultados
        OutputFunctions::writeOutput(output);
        
        // informacoes de debug (apenas em modo debug)
        if (DEBUG_MODE) {
            DebugFunctions::printDebugInfo(input, tree);
        }
        
        return 0;
        
    } catch (const exception& e) {
        ErrorHandling::handleError(string("excecao capturada: ") + e.what());
    } catch (...) {
        ErrorHandling::handleError("erro desconhecido");
    }
    
    return 1;
}

// TODO: implementar funcoes auxiliares conforme necessario
// funcoes que ainda precisam ser implementadas:
// - BSPTree::build()
// - BSPTree::querySegment()
// - algoritmos de intersecao geometrica
// - algoritmos de construcao da arvore bsp
// - metodos de debug e estatisticas 