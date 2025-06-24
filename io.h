#ifndef IO_H
#define IO_H

#include <vector>
#include <string>
#include "bsp.h"

// estrutura para armazenar dados de entrada do problema
struct BSPInput {
    std::vector<Point3D> points;        // lista de pontos (1-based indexing)
    std::vector<Triangle> triangles;    // lista de triangulos
    std::vector<Segment> segments;      // lista de segmentos
    
    // limpa todos os dados
    void clear();
    
    // retorna numero de pontos (sem contar o ponto dummy)
    int getPointCount() const;
};

// estrutura para armazenar resultados da consulta
struct BSPOutput {
    std::vector<std::vector<int>> results;  // para cada segmento, lista de triangulos intersectados
    
    // limpa todos os resultados
    void clear();
    
    // adiciona resultado para um segmento
    void addResult(const std::vector<int>& triangle_ids);
    
    // retorna numero de consultas processadas
    int getQueryCount() const;
};

// funcoes de entrada de dados
namespace InputFunctions {
    // le dados completos da entrada padrao
    bool readInput(BSPInput& input);
    
    // valida os dados de entrada
    bool validateInput(const BSPInput& input);
    
    // funcoes auxiliares privadas
    bool readHeader(int& n, int& t, int& l);
    bool readPoints(BSPInput& input, int n);
    bool readTriangles(BSPInput& input, int t, int n);
    bool readSegments(BSPInput& input, int l);
}

// funcoes de saida de dados
namespace OutputFunctions {
    // escreve resultados na saida padrao
    void writeOutput(const BSPOutput& output);
    
    // escreve uma linha de resultado
    void writeResultLine(const std::vector<int>& triangle_ids);
    
    // formata uma linha de resultado como string
    std::string formatResult(const std::vector<int>& triangle_ids);
}

// funcoes de debug e informacoes
namespace DebugFunctions {
    // imprime informacoes de debug (apenas se DEBUG estiver definido)
    void printDebugInfo(const BSPInput& input, const BSPTree& tree);
    
    // imprime estatisticas da entrada
    void printInputStats(const BSPInput& input);
    
    // imprime estatisticas da arvore
    void printTreeStats(const BSPTree& tree);
}

// funcoes de tratamento de erros
namespace ErrorHandling {
    // trata erro e encerra o programa
    void handleError(const std::string& message);
    
    // trata erro com codigo de saida customizado
    void handleError(const std::string& message, int exit_code);
    
    // verifica se entrada esta em estado valido
    bool checkInputState();
}

#endif // IO_H 