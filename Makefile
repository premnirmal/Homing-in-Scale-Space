CPPFLAGS= -g

# OPENCV
CPPFLAGS+=`pkg-config --cflags opencv`
LDLIBS=`pkg-config --libs opencv`

# ARIA
CPPFLAGS+=-I/usr/local/Aria/include
LDLIBS+=-L/usr/local/Aria/lib -lAria -lArNetworking -lpthread -ldl -lrt

#Eigen
CPPFLAGS+=`pkg-config --cflags eigen3`

# --------------------- Code modules ----------------------------

# Object files
OBJ = defs.h util.c vh.cpp

# Definitions
#DEFS = defs.h

# ------------------------ Rules --------------------------------

vh: ${OBJ}
	g++ ${CPPFLAGS} ${OBJ} ${LDLIBS} -o $@

clean: 
	rm -f *~ data/image* data/matched* data/temp*

# Implicit rule used by Gnu Make: $(CC) -c $(CPPFLAGS) $(CFLAGS)
#${OBJ}: ${DEFS}
