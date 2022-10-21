LIBDIR = -L ./
INCLUDEDIR = -I ./

EXE = output

all:
	clang++ -arch x86_64 -std=c++17 -lSDL2 -Wall $(LIBDIR) $(INCLUDEDIR) *.cpp -o $(EXE)
clean:
	-rm $(EXE)