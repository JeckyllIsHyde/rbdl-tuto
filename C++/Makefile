CXX=g++
CXXFLAGS=-I . -I /usr/local/include/eigen3
LIBS=-lrbdl
DEPS=PhysicsEngine.h Animation.h
ODIR=obj
_OBJ=sketchPhysicsEngine001.o PhysicsEngine.o Animation.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

${ODIR}/%.o: %.cpp ${DEPS}
	@echo Compilling an object $@
	$(CXX) -c -o $@ $< ${CXXFLAGS}

bin/sketchPhysicsEngine: ${OBJ}
	@echo Generating executable $@
	${CXX} -o $@ $^ ${CXXFLAGS} ${LIBS}


.PHONY: clean

clean:
	rm -f ${ODIR}/*.o
