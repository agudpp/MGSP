CC = g++

# define any compile-time flags
CFLAGS = -Wall -g -std=c++11 -ftree-vectorize -O3 -ffast-math -DDEBUG


# define any directories containing header files other than /usr/include
#
INCLUDES = -I.
LFLAGS = -L/usr/local/lib/
# for test we need the UnitTest++ lib
LIBS = -lUnitTest++

# define the C++ source files
SRCS = MultiGridSpacePartition.cpp mgsp_test.cpp

OBJS = $(SRCS:.cpp=.o)

# define the executable file 
MAIN = mgsp

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all:    $(MAIN)
	@echo  Done

$(MAIN): $(OBJS) 
	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS) 
	

# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
.cpp.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	$(RM) *.o *~ $(MAIN)

depend: $(SRCS)
	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it
