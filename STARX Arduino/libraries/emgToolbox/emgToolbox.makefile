
# Run 'make -f emgToolbox.makefile' to compile.
# Run './emgToolbox' to see output.

src = $(wildcard *.cpp)
obj = $(src:.c=.o)
CPPFLAGS = -std=c++14 -g -O3

emgToolbox: $(obj)
	$(CXX) -o $@ $^ $(CPPFLAGS)
.PHONY: clean
clean:
	rm -f $(obj) emgToolbox
