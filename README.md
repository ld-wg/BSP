# BSP - Binary Space Partitioning: Documentação Técnica

## Visão Geral

Implementação de árvore BSP (Binary Space Partitioning) para triângulos no R³. O sistema constrói uma BSP com triângulos e usa essa estrutura para encontrar eficientemente quais triângulos são intersectados por segmentos de reta.

## Estrutura do Projeto

```
/
├── main.cpp          # Ponto de entrada, processamento de argumentos, orquestração das fases
├── bsp.h/.cpp        # Estruturas geométricas, árvore BSP, algoritmos de construção e consulta
├── io.h/.cpp         # Parser de entrada, validação, formatação de saída
└── Makefile
```

## Estruturas de Dados

### **Point3D** - Pontos no Espaço 3D

```cpp
struct Point3D {
    double x, y, z;

    // Operações vetoriais
    Point3D operator+(const Point3D& other) const;
    Point3D operator-(const Point3D& other) const;
    Point3D operator*(double scalar) const;

    // Produtos escalar e vetorial
    double dot(const Point3D& other) const;
    Point3D cross(const Point3D& other) const;

    // Magnitude e normalização
    double length() const;
    Point3D normalize() const;
};
```

### **Triangle** - Triângulos no Espaço

```cpp
struct Triangle {
    Point3D vertices[3];  // Vértices A, B, C
    int id;              // ID único (1-based)

    // Geometria do triângulo
    Point3D getNormal() const;
    void getPlaneEquation(double& a, double& b, double& c, double& d) const;
    bool isPointInPlane(const Point3D& point) const;
};
```

- **Cálculo de Normal**: Produto vetorial das arestas
- **Equação do Plano**: Forma ax + by + cz + d = 0
- **Teste de Coplanaridade**: Verificação com tolerância numérica

### **Segment** - Segmentos de Reta

```cpp
struct Segment {
    Point3D start, end;

    Point3D getDirection() const;
    double getLength() const;
};
```

### **PlaneInfo** - Planos Divisórios

```cpp
struct PlaneInfo {
    double a, b, c, d;  // Equação: ax + by + cz + d = 0

    // Classificação espacial
    double classifyPoint(const Point3D& point) const;
    TriangleClassification classifyTriangle(const Triangle& triangle) const;
};
```

**Tipos de Classificação:**

- `FRONT`: Todo o triângulo está na frente do plano
- `BACK`: Todo o triângulo está atrás do plano
- `COPLANAR`: Triângulo está no plano
- `SPANNING`: Triângulo cruza o plano (requer divisão)

### **BSPNode** - Nós da Árvore

```cpp
class BSPNode {
public:
    PlaneInfo plane;                        // Plano divisor (nós internos)
    std::vector<Triangle> triangles;        // Triângulos (folhas)
    std::unique_ptr<BSPNode> front, back;   // Filhos
    bool is_leaf;                           // Tipo do nó
};
```

**Características:**

- **Nós Folha**: Armazenam triângulos diretamente
- **Nós Internos**: Contêm plano divisor e dois filhos
- **Gestão de Memória**: Smart pointers para segurança

### **BSPTree** - Árvore Principal

```cpp
class BSPTree {
private:
    std::unique_ptr<BSPNode> root;

public:
    void build(std::vector<Triangle>& triangles);
    std::vector<int> querySegment(const Segment& segment);
};
```

## Algoritmos

### **Visão Geral do Fluxo do Programa**

**Passo a passo:**

```
1. ENTRADA: Lê pontos 3D, triângulos (índices), segmentos de consulta
   - Parser processa formato específico: header (n,t,l), coordenadas, índices
   - Validação de dados (índices válidos, coordenadas em range esperado)
   - Construção das estruturas Point3D, Triangle, Segment em memória
   ↓
2. CONSTRUÇÃO BSP: Divide triângulos recursivamente usando planos divisórios
   - Escolhe plano divisor (estratégia: primeiro triângulo da lista)
   - Calcula equação do plano: ax + by + cz + d = 0
   - Classifica outros triângulos: front/back/coplanar/spanning
   - Triângulos spanning → algoritmo de splitting cria novos triângulos
   - Cria nó interno com plano, recursão em sublistas front/back
   - Recursão até critério de parada (≤ MAX_TRIANGLES ou profundidade ≥ MAX_DEPTH)
   - Folhas armazenam triângulos diretamente (sem plano divisor)
   ↓
3. CONSULTAS: Para cada segmento, percorre árvore BSP otimizadamente
   - Classifica endpoints do segmento relativo aos planos (distância assinada)
   - Decisão de travessia: visita front/back baseado na classificação
   - Poda espacial: evita subárvores impossíveis de intersectar
   - Nas folhas: aplica Möller-Trumbore para cada triângulo
   - Coleta IDs dos triângulos intersectados
   ↓
4. SAÍDA: Lista de triângulos intersectados por cada segmento (ordenados)
   - Remoção de duplicatas (sort + unique)
   - Formatação: número de intersecções seguido dos IDs crescentes
   - Output final: uma linha por segmento de consulta
```

**Analogia:** Como um sistema de biblioteca inteligente - primeiro categoriza todos os livros por assunto recursivamente (BSP construction), criando um índice hierárquico. Depois, para cada consulta, navega apenas pelas seções relevantes (spatial pruning), ignorando prateleiras impossíveis. Finalmente, examina apenas os livros candidatos nas prateleiras visitadas (triangle intersection), retornando a lista ordenada de resultados.

### 1. Construção da BSP

**`void buildTreeRecursive(BSPNode* node, vector<Triangle>& triangles, int depth)`**

Algoritmo recursivo de divisão espacial que constrói a árvore BSP:

**Pseudo-código:**

```
FUNCTION buildTreeRecursive(node, triangles, depth)
    IF triangles.size <= MAX_TRIANGLES OR depth >= MAX_DEPTH THEN
        node.makeLeaf(triangles)
        RETURN
    END IF

    plane = selectSplittingPlane(triangles[0])
    front_list, back_list, coplanar_list = classifyTriangles(triangles, plane)

    node.makeInternal(plane)
    buildTreeRecursive(node.front, front_list, depth+1)
    buildTreeRecursive(node.back, back_list, depth+1)
END FUNCTION
```

**Estratégia**: Usa o plano do primeiro triângulo como divisor (First Triangle Heuristic). Simples e eficiente, garante progresso na divisão.

### 2. Divisão de Triângulos Spanning

**`void splitSpanningTriangle(const Triangle& triangle, const PlaneInfo& plane, vector<Triangle>& front, vector<Triangle>& back)`**

Algoritmo que divide triângulos que cruzam planos divisórios:

**Classificação de vértices:**

- Calcula distância de cada vértice ao plano (positiva=front, negativa=back, ~zero=coplanar)
- Identifica configuração: 1-front-2-back, 2-front-1-back, ou vértices no plano

**Caso 1: Um vértice na frente, dois atrás**

```
     A (front)
     /\
    /  \
   I1---I2  <- Intersecções com o plano
  /      \
 B-------C (back)

Resultado:
- 1 triângulo na frente: (A, I1, I2)
- 2 triângulos atrás: (B, C, I1) e (C, I2, I1)
```

**Caso 2: Dois vértices na frente, um atrás**

```
 A-------B (front)
  \      /
   I1---I2  <- Intersecções
    \  /
     C (back)

Resultado:
- 2 triângulos na frente: (A, B, I1) e (B, I2, I1)
- 1 triângulo atrás: (C, I1, I2)
```

**Caso 3: Vértice no plano**

- **1 no plano, 1 frente, 1 atrás**: Divide em 2 triângulos
- **2 no plano**: Triângulo inteiro vai para um lado

**`bool calculateEdgePlaneIntersection(const Point3D& p1, const Point3D& p2, const PlaneInfo& plane, Point3D& intersection)`**

Calcula intersecção aresta-plano usando equação paramétrica da reta e equação implícita do plano:

**Fundamento Matemático:**

- **Equação do Plano**: ax + by + cz + d = 0, onde (a,b,c) é o vetor normal
- **Equação Paramétrica da Reta**: P(t) = p1 + t\*(p2-p1), onde t ∈ [0,1] para o segmento
- **Intersecção**: Substituir P(t) na equação do plano e resolver para t

**Por que essa abordagem funciona:**

A intersecção aresta-plano é fundamentalmente um problema de **encontrar onde uma linha cruza uma superfície infinita**:

**1. Interpretação Geométrica:**

- **Linha como trajetória**: Imagine uma partícula que se move ao longo da aresta com velocidade constante `direction`
- **Plano como barreira**: O plano divide o espaço em duas regiões (frente/atrás da normal)
- **Intersecção como "momento do cruzamento"**: Quando a partícula passa de uma região para outra

**2. O papel fundamental do produto escalar:**

```
denominador = normal · direction
```

Este valor determina **como** a reta se aproxima do plano:

- **= 0**: Reta paralela ao plano (nunca cruza ou está inteiramente no plano)
- **> 0**: Reta se aproxima "contra" a normal (cruzamento frontal)
- **< 0**: Reta se aproxima "a favor" da normal (cruzamento traseiro)
- **|denominador| grande**: Cruzamento quase perpendicular (intersecção "direta")
- **|denominador| pequeno**: Cruzamento quase tangencial (numericamente instável)

**3. Significado físico do parâmetro t:**

```
t = -(normal·p1 + d) / (normal·direction)
```

- **Numerador**: Distância **assinada** de p1 ao plano (positiva se p1 está do lado da normal)
- **Denominador**: "Velocidade de aproximação" ao plano na direção da normal
- **Resultado t**: "Tempo" necessário para chegar ao plano desde p1

**4. Validação geométrica:**

- **t ∈ [0,1]**: Intersecção dentro do segmento [p1,p2]
- **t < 0**: Intersecção "atrás" de p1 (fora do segmento)
- **t > 1**: Intersecção "além" de p2 (fora do segmento)

**Analogia intuitiva**: É como calcular quando um projétil (que segue a aresta) vai atravessar uma parede (o plano). O produto escalar nos diz se o projétil está indo em direção à parede e quão diretamente está mirando nela.

**Derivação:**

```
Substituindo P(t) no plano:
a(p1.x + t*dir.x) + b(p1.y + t*dir.y) + c(p1.z + t*dir.z) + d = 0

Reorganizando:
a*p1.x + b*p1.y + c*p1.z + d + t*(a*dir.x + b*dir.y + c*dir.z) = 0

Isolando t:
t = -(a*p1.x + b*p1.y + c*p1.z + d) / (a*dir.x + b*dir.y + c*dir.z)
t = -(normal·p1 + d) / (normal·direction)
```

**Pseudo-código:**

```
FUNCTION calculateIntersection(p1, p2, plane)
    direction = p2 - p1
    denominator = dot(plane.normal, direction)

    IF |denominator| < EPSILON THEN
        RETURN false  // Aresta paralela ao plano (denominador = 0)
    END IF

    numerator = -(dot(plane.normal, p1) + plane.d)
    t = numerator / denominator

    IF t < 0 OR t > 1 THEN
        RETURN false  // Intersecção fora do segmento [p1,p2]
    END IF

    intersection = p1 + t * direction
    RETURN true
END FUNCTION
```

**Casos Especiais:**

- **denominator ≈ 0**: Reta paralela ao plano (normal ⊥ direction)
- **t < 0**: Intersecção "atrás" de p1 (fora do segmento)
- **t > 1**: Intersecção "além" de p2 (fora do segmento)
- **t = 0**: Intersecção exatamente em p1
- **t = 1**: Intersecção exatamente em p2

### 3. Consultas de Intersecção

**`void querySegmentRecursive(BSPNode* node, const Segment& segment, vector<int>& results)`**

Algoritmo de travessia que encontra triângulos intersectados por um segmento:

**Pseudo-código:**

```
FUNCTION querySegment(node, segment, results)
    IF node == NULL THEN RETURN

    IF node.isLeaf THEN
        FOR each triangle IN node.triangles DO
            IF segmentIntersectsTriangle(segment, triangle) THEN
                results.add(triangle.id)
            END IF
        END FOR
        RETURN
    END IF

    start_dist = classifyPoint(segment.start, node.plane)
    end_dist = classifyPoint(segment.end, node.plane)

    visit_front = (start_dist >= -EPSILON) OR (end_dist >= -EPSILON)
    visit_back = (start_dist <= EPSILON) OR (end_dist <= EPSILON)

    IF visit_front THEN querySegment(node.front, segment, results)
    IF visit_back THEN querySegment(node.back, segment, results)
END FUNCTION
```

**Estratégia**: Classifica endpoints do segmento e visita apenas subárvores relevantes, otimizando a busca.

### **Como Funciona o Spatial Pruning na BSP**

O **spatial pruning** (poda espacial) é o mecanismo fundamental que torna a BSP eficiente. Em vez de testar o segmento contra todos os triângulos da cena (força bruta O(n)), a BSP elimina regiões inteiras do espaço que são impossíveis de intersectar.

#### **Conceito Central: Classificação de Endpoints**

Para cada nó interno da BSP, classificamos os **dois endpoints** do segmento de consulta relativo ao plano divisor:

```cpp
start_dist = classifyPoint(segment.start, node.plane)   // Distância assinada do início
end_dist = classifyPoint(segment.end, node.plane)       // Distância assinada do fim
```

**Interpretação das distâncias:**

- **> +EPSILON**: Ponto claramente na frente do plano
- **< -EPSILON**: Ponto claramente atrás do plano
- **[-EPSILON, +EPSILON]**: Ponto no plano (considerando tolerância numérica)

#### **Decisão de Travessia: Lógica de Poda**

```cpp
visit_front = (start_dist >= -EPSILON) OR (end_dist >= -EPSILON)
visit_back = (start_dist <= EPSILON) OR (end_dist <= EPSILON)
```

**Casos de poda espacial:**

**1. Ambos endpoints atrás do plano:**

```
start_dist < -EPSILON AND end_dist < -EPSILON
→ visit_front = false, visit_back = true
→ PODA: Subárvore front é completamente ignorada
```

**2. Ambos endpoints na frente do plano:**

```
start_dist > +EPSILON AND end_dist > +EPSILON
→ visit_front = true, visit_back = false
→ PODA: Subárvore back é completamente ignorada
```

**3. Segmento cruza o plano:**

```
(start_dist * end_dist) < 0  // Sinais opostos
→ visit_front = true, visit_back = true
→ SEM PODA: Ambas subárvores devem ser visitadas
```

**4. Endpoint no plano:**

```
|start_dist| <= EPSILON OR |end_dist| <= EPSILON
→ Ambas subárvores visitadas (caso conservador)
→ PODA MÍNIMA: Evita falsos negativos por erro numérico
```

#### **Exemplo Visual de Spatial Pruning**

```
                 Plano Divisor
                      |
    Subárvore BACK    |    Subárvore FRONT
                      |
         T1           |         T4
                      |
         T2    -------+-------  ← Segmento de consulta
                      |
         T3           |         T5
                      |
```

**Análise:**

- **Segmento**: Início atrás (-), fim na frente (+)
- **Classificação**: `start_dist < 0`, `end_dist > 0`
- **Decisão**: `visit_front = true`, `visit_back = true`
- **Resultado**: Testa triângulos em ambas subárvores (T1,T2,T3,T4,T5)

**Contra-exemplo (com poda):**

```
                 Plano Divisor
                      |
    Subárvore BACK    |    Subárvore FRONT
                      |
         T1           |         T4
                      |
         T2           |         T5
                      |
         T3    -------+-------- ← Segmento só no lado BACK
                      |
```

**Análise:**

- **Segmento**: Ambos endpoints atrás
- **Classificação**: `start_dist < 0`, `end_dist < 0`
- **Decisão**: `visit_front = false`, `visit_back = true`
- **PODA**: Triângulos T4,T5 nunca são testados!
- **Eficiência**: 40% menos testes de intersecção

#### **Profundidade da Poda: Efeito Cascata**

A poda spatial funciona **recursivamente**, criando um efeito multiplicativo:

```
Nível 0: 1000 triângulos
    ↓ (poda elimina 50% da cena)
Nível 1: 500 triângulos candidatos
    ↓ (poda elimina 50% do restante)
Nível 2: 250 triângulos candidatos
    ↓ (poda elimina 50% do restante)
Nível 3: 125 triângulos candidatos
    ↓
...
Folhas: ~10-20 triângulos efetivamente testados
```

**Ganho teórico**: De O(n) para O(log n) testes na média.

#### **Casos Especiais de Poda**

**Segmento muito curto:**

- Ambos endpoints na mesma região
- Poda máxima: ~50% da árvore eliminada por nível

**Segmento atravessando toda a cena:**

- Cruza muitos planos
- Poda mínima: visita muitas subárvores

**Segmento tangente a planos:**

- Endpoints próximos a planos (dentro de EPSILON)
- Poda conservadora: evita falsos negativos

#### **Robustez Numérica na Poda**

```cpp
// Tolerância conservadora para evitar falsos negativos
visit_front = (start_dist >= -EPSILON) OR (end_dist >= -EPSILON)
visit_back = (start_dist <= EPSILON) OR (end_dist <= EPSILON)
```

**Por que usar EPSILON:**

- **Problema**: Erros de ponto flutuante podem classificar incorretamente pontos muito próximos ao plano
- **Solução**: Zona de "incerteza" de ±EPSILON onde assumimos que o ponto pode estar em qualquer lado
- **Trade-off**: Menos poda (mais conservador) vs maior robustez (sem falsos negativos)

**Exemplo crítico:**

```
Ponto teoricamente no plano: distance = 1e-15 (erro numérico)
Sem EPSILON: classificado como "frente" → poda incorreta da subárvore back
Com EPSILON: classificado como "incerto" → ambas subárvores visitadas
```

#### **Eficiência da Poda Espacial**

**Métrica**: Fração da árvore efetivamente visitada

- **Melhor caso**: Segmento em região isolada → ~log(n) nós visitados
- **Caso médio**: Segmento atravessa algumas regiões → ~√n nós visitados
- **Pior caso**: Segmento atravessa toda a cena → ~n nós visitados

**Exemplo prático:**

- **Cena**: 1000 triângulos, BSP com profundidade 10
- **Sem poda**: 1000 testes de intersecção ray-triângulo
- **Com poda**: ~20-50 testes (speedup de 20-50x)

O spatial pruning transforma a BSP de uma simples estrutura hierárquica em um **acelerador espacial inteligente**, permitindo consultas sub-lineares mesmo em cenas complexas.

**`bool rayIntersectsTriangle(const Point3D& origin, const Point3D& direction, const Triangle& triangle, double& t)`**

**Conexão com algoritmo anterior:** Enquanto `calculateEdgePlaneIntersection` resolve o problema 1D (ponto em linha), Möller-Trumbore resolve o problema 2D completo: determinar se um ray atinge o **interior** de um triângulo.

**Por que Möller-Trumbore é elegante:**

A intersecção ray-triângulo tradicionalmente requer **dois passos**:

1. **Ray-Plano**: Encontrar onde o ray cruza o plano do triângulo
2. **Ponto-em-Triângulo**: Verificar se a intersecção está dentro das bordas

Möller-Trumbore faz **ambos simultaneamente** usando coordenadas baricêntricas, evitando cálculos intermediários e armazenamento de equações de plano.

**Fundamentação Matemática:**

Qualquer ponto P dentro de um triângulo pode ser expresso como:

```
P = (1-u-v)·V0 + u·V1 + v·V2
```

onde u,v são **coordenadas baricêntricas** com restrições:

- `u ≥ 0, v ≥ 0, u+v ≤ 1` (dentro do triângulo)
- `u+v = 1` (na aresta oposta a V0)
- `u = 0` (na aresta V0-V2)
- `v = 0` (na aresta V0-V1)

#### **Como Funcionam Coordenadas Baricêntricas**

**Conceito Central**: Em vez de usar coordenadas cartesianas (x,y,z), expressamos a posição de um ponto como **combinação ponderada** dos vértices do triângulo.

**Interpretação Física - "Centro de Massa":**

Imagine o triângulo como uma placa rígida com massas nos vértices:

- **Massa em V0**: `w = 1-u-v`
- **Massa em V1**: `u`
- **Massa em V2**: `v`

O ponto P é onde ficaria o **centro de massa** do sistema. As coordenadas baricêntricas são as **proporções de massa** em cada vértice.

**Interpretação Geométrica - "Áreas Proporcionais":**

```
Triângulo ABC com ponto P interno:

        A
       /|\
      / | \
     /  P  \    u = Área(PBC) / Área(ABC)
    /   |   \   v = Área(APC) / Área(ABC)
   /    |    \  w = Área(APB) / Área(ABC)
  B-----------C
```

Cada coordenada baricêntrica é a **proporção da área** do sub-triângulo oposto ao vértice correspondente.

**Propriedades Fundamentais:**

1. **Conservação**: `u + v + w = 1` (soma das proporções = 100%)
2. **Não-negatividade**: `u,v,w ≥ 0` para pontos internos
3. **Linearidade**: Interpolação de qualquer propriedade é linear

**Casos Especiais Importantes:**

```
Posição no Triângulo     →  Coordenadas (u,v,w)
────────────────────────    ──────────────────
Centro de V0             →  (0, 0, 1)
Centro de V1             →  (1, 0, 0)
Centro de V2             →  (0, 1, 0)
Centro do triângulo      →  (1/3, 1/3, 1/3)
Meio da aresta V0-V1     →  (1/2, 0, 1/2)
Meio da aresta V1-V2     →  (1/2, 1/2, 0)
Meio da aresta V0-V2     →  (0, 1/2, 1/2)
```

**Por que são Úteis em Graphics:**

- **Interpolação Natural**: Textura, cor, normais se interpolam linearmente
- **Teste de Inclusão**: Ponto interno ⟺ todas coordenadas ≥ 0
- **Independente de Orientação**: Funcionam em qualquer sistema de coordenadas
- **Numericamente Estável**: Bem condicionadas para triângulos não-degenerados

**Exemplo Prático:**

```
Triângulo com cores nos vértices:
V0 = vermelho   (1,0,0)
V1 = verde      (0,1,0)
V2 = azul       (0,0,1)

Ponto P com coordenadas baricêntricas (0.5, 0.3, 0.2):
Cor em P = 0.5*vermelho + 0.3*verde + 0.2*azul = (0.5, 0.3, 0.2)
```

Esta interpolação linear é **exatamente** o que shaders fazem para cores, texturas e normais!

**Insight Geométrico Chave:**

Em vez de resolver para coordenadas XYZ do ponto de intersecção, resolvemos diretamente para (t,u,v):

- **t**: distância ao longo do ray (como no algoritmo anterior)
- **u,v**: posição dentro do triângulo (coordenadas baricêntricas)

**Por que isso funciona:**

```
Ray:      R(t) = origin + t·direction
Triângulo: T(u,v) = V0 + u·(V1-V0) + v·(V2-V0)
Intersecção: R(t) = T(u,v)
```

Isso gera um sistema linear 3×3 que pode ser resolvido via **Regra de Cramer** usando determinantes (produtos misto de vetores).

**Interpretação Física dos Cálculos:**

```
h = cross(direction, edge2)    // Normal ao plano que contém direction e edge2
a = dot(edge1, h)              // Volume orientado do paralelepípedo [edge1, direction, edge2]
```

- **a ≈ 0**: Ray paralelo ao triângulo (volume = 0)
- **a > 0**: Ray "entra" pela frente do triângulo
- **a < 0**: Ray "entra" por trás do triângulo

**Pseudo-código:**

```
FUNCTION rayTriangleIntersect(origin, direction, triangle)
    edge1 = triangle.v1 - triangle.v0
    edge2 = triangle.v2 - triangle.v0

    // Teste de paralelismo usando produto misto
    h = cross(direction, edge2)
    a = dot(edge1, h)
    IF |a| < EPSILON THEN RETURN false  // Ray paralelo ao triângulo

    // Calcular coordenada baricêntrica u
    inv_det = 1.0 / a
    s = origin - triangle.v0
    u = inv_det * dot(s, h)
    IF u < 0 OR u > 1 THEN RETURN false  // Fora das bordas u

    // Calcular coordenada baricêntrica v
    q = cross(s, edge1)
    v = inv_det * dot(direction, q)
    IF v < 0 OR u+v > 1 THEN RETURN false  // Fora das bordas v

    // Calcular distância t ao longo do ray
    t = inv_det * dot(edge2, q)
    RETURN t > EPSILON  // Intersecção válida à frente do ray
END FUNCTION
```

**Validação Geométrica Progressiva:**

1. **Paralelismo**: `|a| < EPSILON` → ray paralelo ao plano do triângulo
2. **Primeira coordenada**: `u ∈ [0,1]` → entre as arestas v0-v2 e linha paralela passando por v1
3. **Segunda coordenada**: `v ∈ [0,1]` e `u+v ≤ 1` → dentro do triângulo propriamente dito
4. **Direção do ray**: `t > 0` → intersecção à frente (não atrás) da origem

**Vantagens sobre métodos tradicionais:**

- **Sem equação de plano**: Não precisa armazenar/calcular (a,b,c,d)
- **Menos operações**: Um só sistema linear em vez de dois passos separados
- **Coordenadas baricêntricas grátis**: Úteis para interpolação de textura/normais
- **Numericamente estável**: Usa produtos misto que são geometricamente robustos

**Eficiência**: O(1) por teste, robusto para casos degenerados, conexão direta com shading via coordenadas baricêntricas.

#### **Conexões Entre os Algoritmos da BSP**

**Hierarquia de Complexidade Geométrica:**

```
1D: calculateEdgePlaneIntersection  →  "Onde uma linha cruza um plano?"
2D: rayIntersectsTriangle           →  "Onde um ray atinge um triângulo?"
3D: querySegmentRecursive          →  "Quais triângulos um segmento intersecta?"
```

**Reutilização de Conceitos:**

- **Produto Escalar**: `calculateEdgePlaneIntersection` usa `normal·direction` para detectar paralelismo; Möller-Trumbore usa produtos escalares similares para coordenadas baricêntricas
- **Parâmetro t**: Ambos algorithms calculam `t` como "distância ao longo da reta", mas com interpretações diferentes (segmento vs ray infinito)
- **Tolerância EPSILON**: Ambos usam a mesma estratégia para robustez numérica em casos degenerados

**Fluxo de Dados na BSP:**

```
querySegmentRecursive
    ↓ (para cada triângulo em folhas)
rayIntersectsTriangle (Möller-Trumbore)
    ↓ (internamente, conceptualmente similar a)
calculateEdgePlaneIntersection
```

**Otimizações Compartilhadas:**

- **Early Termination**: Ambos retornam `false` rapidamente em casos impossíveis
- **Lazy Evaluation**: Möller-Trumbore calcula `u` antes de `v`, evitando `v` se `u` já falha
- **Numerical Stability**: Ambos tratam `denominador ≈ 0` como caso especial

**Por que essa Arquitetura:**

1. **Modularidade**: `calculateEdgePlaneIntersection` é reutilizável em outros contextos (splitting de triângulos)
2. **Performance**: Möller-Trumbore é otimizado especificamente para ray-triângulo (caso mais comum)
3. **Clareza**: Separação de responsabilidades torna o código mais legível e debugável

## Análise de Complexidade

### **Complexidade de Construção**

- **Caso médio**: O(n log n) - boa distribuição, profundidade log n
- **Pior caso**: O(n²) - árvore desbalanceada, profundidade n

**Detalhamento:**

- **Seleção de Planos**: O(1) por nível (First Triangle Heuristic)
- **Classificação**: O(n) por nível
- **Splitting**: O(k) onde k é número de triângulos spanning
- **Profundidade**: O(log n) com boa distribuição
- **Total**: O(n log n)

**Complexidade de Espaço:**

- **Melhor caso**: O(n) - poucos triângulos spanning, nós balanceados
- **Pior caso**: O(n²) - muitos splitting, triângulos duplicados

### **Complexidade de Consulta**

#### **Caso Médio: O(log n) por segmento**

- **Travessia**: O(log n) nós visitados em árvore balanceada
- **Testes de Intersecção**: O(k) onde k é triângulos nas folhas visitadas
- **Total**: O(log n + k)

#### **Pior Caso: O(n) por segmento**

- Segmento atravessa toda a cena
- Visita todas as folhas da árvore
- Total: O(n)

## Como Usar

**Compilação:**

```bash
make          # Normal
make debug    # Com debug
make clean    # Limpeza
```

**Execução:**

```bash
./bsp < entrada.txt > saida.txt    # Normal
./bsp --debug < entrada.txt        # Debug
```

**Formato entrada:**

```
n t l
x₁ y₁ z₁
x₂ y₂ z₂
...
xₙ yₙ zₙ
i₁ j₁ k₁
i₂ j₂ k₂
...
iₜ jₖ kₜ
xa₁ ya₁ za₁ xb₁ yb₁ zb₁
xa₂ ya₂ za₂ xb₂ yb₂ zb₂
...
xaₗ yaₗ zaₗ xbₗ ybₗ zbₗ
```

**Onde:**

- `n`: número de pontos
- `t`: número de triângulos
- `l`: número de segmentos
- Coordenadas: números inteiros entre 1 e 99
- Índices: 1-based para pontos e triângulos

### **Formato de Saída**

```
k₁ id₁ id₂ ... idₖ₁
k₂ id₁ id₂ ... idₖ₂
...
kₗ id₁ id₂ ... idₖₗ
```

**Onde:**

- `kᵢ`: número de triângulos intersectados pelo segmento i
- `id₁ id₂ ... idₖᵢ`: IDs dos triângulos, **ordenados crescentemente**

## Considerações Técnicas

**Tolerâncias Numéricas:**

- EPSILON = 1e-9 para classificação de pontos, detecção de intersecções e casos degenerados
- Tratamento robusto de casos degenerados (rays paralelos, intersecções tangentes)

**Gestão de Memória:**

- Smart pointers para RAII e exception safety
- Destruição automática da árvore
- Sem vazamentos em caso de exceção

**Otimizações Implementadas:**

- Remoção de duplicatas com sort+unique
- Early termination na travessia BSP
- Indexação 1-based eficiente
- Reserva de memória para containers
