CC=g++
CFLAGS=-Wall -Wextra -Wno-unused-parameter -I.
LDFLAGS=

TARGETS=tests
SOURCES=$(shell echo *.cpp)	
COMMON_OBJECTS=$(SOURCES:.cpp=.o)

all: $(TARGETS)

tests: $(COMMON_OBJECTS)
	$(CC) $(CFLAGS) $^ -o $@ -I. $(LDFLAGS)

clean:
	rm -f $(TARGETS) *.o .depend *~

.depend: *.[ch]
	$(CC) -MM $(SOURCES) >.depend

-include .depend

.PHONY: clean all
