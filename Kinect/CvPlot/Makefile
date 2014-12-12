TARGET = plots 

SRC = \
	cvplot.cpp \
	plot_sample.cpp \

INCLUDE = \
    -I. \

LIB = \

DEFINE = \

CC = g++ 

#CFLAGS = `pkg-config opencv --cflags` \
	-lm -lml -lcvaux -lhighgui -lcv -lcxcore \
	-O3 \
	-Wall \

CFLAGS = `pkg-config opencv --cflags` `pkg-config opencv --libs`\
	-O3 \
	-Wall \

#CFLAGS = `pkg-config opencv --cflags` `pkg-config opencv --libs`\
	-g

OBJ = $(patsubst %.cpp,%.o,$(filter %.cpp,$(SRC)))

.SUFFIXES: .cpp .o

.cpp.o:
	$(CC) $(CFLAGS) $(DEFINE) $(INCLUDE) -c $< -o $@

all: $(TARGET)


$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LIB) $(CFLAGS)

clean:
	rm $(TARGET) $(OBJ)

