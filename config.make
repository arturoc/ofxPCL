# add custom variables to this file

# OF_ROOT allows to move projects outside apps/* just set this variable to the
# absoulte path to the OF root folder

OF_ROOT = ../../..


# USER_CFLAGS allows to pass custom flags to the compiler
# for example search paths like:
# USER_CFLAGS = -I src/objects

USER_CFLAGS = $(shell pkg-config pcl_common-1.3 pcl_surface-1.3 pcl_filters-1.3 openni-dev eigen3 --cflags)


# USER_LDFLAGS allows to pass custom flags to the linker
# for example libraries like:
# USER_LD_FLAGS = libs/libawesomelib.a

USER_LDFLAGS = 


# use this to add system libraries for example:
# USER_LIBS = -lpango
 
USER_LIBS = -lusb-1.0 -lpcl_surface -lpcl_registration -lpcl_sample_consensus -lpcl_features -lcminpack -lm -lpcl_kdtree -lpcl_range_image -lpcl_filters -lpcl_search -lflann_cpp -lpcl_common -lpcl_io -lOpenNI -lpcl_octree


# change this to add different compiler optimizations to your project

USER_COMPILER_OPTIMIZATION = -march=native -mtune=native -Os


EXCLUDE_FROM_SOURCE="bin,.xcodeproj,obj,.git"
