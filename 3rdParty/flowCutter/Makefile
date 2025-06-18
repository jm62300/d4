CC := g++
SRC_DIR ?= src
OBJ_DIR ?= obj
DEBUG ?= 1

ifeq '$(DEBUG)' '1'
CFLAGS ?= -Wall -MMD -g -Wno-sign-compare -fPIC
else
CFLAGS ?= -Wall -MMD -O3 -DNDEBUG -Wno-sign-compare -fPIC
endif

LDFLAGS= 

OBJS := $(shell find $(SRC_DIR) -name "*.cpp" |sed 's/.cpp$$/.o/g' | sed 's/$(SRC_DIR)/$(OBJ_DIR)/g')
DEPS := $(OBJS:.o=.d)
TARGET ?= libflowCutter.a

all: $(TARGET) 

libflowCutter.a: $(OBJS)	
	ar rcs $@ $(OBJS)


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)

mrproper: clean
	rm -f $(TARGET)


-include $(DEPS)

