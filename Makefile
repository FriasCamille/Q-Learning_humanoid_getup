# Compiler settings - Can be customized.
CC = g++-11
CXXFLAGS = -std=c++20 -Wall -I/usr/include/mujoco -O3 -march=native -flto
LDFLAGS = -lmujoco -lGL -lGLEW -lGLU -lglfw -lyaml-cpp -pthread

# Makefile settings - Can be customized.
APPNAME = getup
EXT = .cpp
SRCDIR = src
OBJDIR = obj

############## No cambies desde aquí hacia abajo #####################

SRC = $(wildcard $(SRCDIR)/*$(EXT))
OBJ = $(SRC:$(SRCDIR)/%$(EXT)=$(OBJDIR)/%.o)
DEP = $(OBJ:$(OBJDIR)/%.o=%.d)
RM = rm

########################################################################
####################### Targets beginning here #########################
########################################################################

all: $(APPNAME)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(APPNAME): $(OBJDIR) $(OBJ)
	$(CC) $(CXXFLAGS) -o $@ $(OBJ) $(LDFLAGS)

%.d: $(SRCDIR)/%$(EXT)
	@$(CPP) $(CFLAGS) $< -MM -MT $(@:%.d=$(OBJDIR)/%.o) >$@

-include $(DEP)

$(OBJDIR)/%.o: $(SRCDIR)/%$(EXT)
	$(CC) $(CXXFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	$(RM) $(OBJ) $(DEP) $(APPNAME)
