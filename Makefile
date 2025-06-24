# makefile para o projeto bsp
# compila o executavel bsp a partir dos arquivos fonte

# compilador e flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
DEBUG_FLAGS = -DDEBUG -g

# arquivos fonte e objeto
SOURCES = main.cpp bsp.cpp io.cpp
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = bsp

# regra principal
all: $(TARGET)

# compilacao do executavel
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

# compilacao dos arquivos objeto
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# versao de debug
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: $(TARGET)

# limpeza dos arquivos gerados
clean:
	rm -f $(OBJECTS) $(TARGET)

# reinstalacao completa
rebuild: clean all

# regras que nao geram arquivos
.PHONY: all clean debug rebuild

# dependencias dos headers
main.o: main.cpp bsp.h io.h
bsp.o: bsp.cpp bsp.h
io.o: io.cpp io.h bsp.h 