
QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += /usr/local/include/pcl-1.8

INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /usr/include/ni

INCLUDEPATH += /usr/include/vtk-5.8

 #CUDA配置，具体按照你系统环境更改
CUDA_SDK = "/usr/local/cuda-7.5/"   # cudaSDK路径
CUDA_DIR = "/usr/local/cuda-7.5/"            # CUDA tookit路径
SYSTEM_NAME = linux         # 自己系统环境 'Win32', 'x64', or 'Win64'
SYSTEM_TYPE = 64            #操作系统位数 '32' or '64',
CUDA_ARCH = sm_30           # cuda架构, for example 'compute_10', 'compute_11', 'sm_10'
NVCC_OPTIONS = --use_fast_math
# include paths
INCLUDEPATH += $$CUDA_DIR/include
# library directories
QMAKE_LIBDIR += $$CUDA_DIR/lib64
CUDA_OBJECTS_DIR = ./
# The following library conflicts with something in Cuda
#QMAKE_LFLAGS_RELEASE = /NODEFAULTLIB:msvcrt.lib
#QMAKE_LFLAGS_DEBUG   = /NODEFAULTLIB:msvcrtd.lib
# Add the necessary libraries

LIBS = -lcudart -lcufft
# The following makes sure all path names (which often include spaces) are put between quotation marks
CUDA_INC = $$join(INCLUDEPATH,'" -I"','-I"','"')
NVCC_LIBS = $$join(CUDA_LIBS,' -l','-l', '')
#LIBS += $$join(CUDA_LIBS,'.so ', '', '.so')
# Configuration of the Cuda compiler
CONFIG(debug, debug|release) {
    # Debug mode
    cuda_d.input = CUDA_SOURCES
    cuda_d.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}_cuda.o
    cuda_d.commands = $$CUDA_DIR/bin/nvcc -D_DEBUG $$NVCC_OPTIONS $$CUDA_INC $$NVCC_LIBS --machine $$SYSTEM_TYPE -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
    cuda_d.dependency_type = TYPE_C
    QMAKE_EXTRA_COMPILERS += cuda_d
}
else {
    # Release mode
    cuda.input = CUDA_SOURCES
    cuda.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}_cuda.o
    cuda.commands = $$CUDA_DIR/bin/nvcc $$NVCC_OPTIONS $$CUDA_INC $$NVCC_LIBS --machine $$SYSTEM_TYPE -arch=$$CUDA_ARCH -O3 -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
    cuda.dependency_type = TYPE_C
    QMAKE_EXTRA_COMPILERS += cuda
}


LIBS += /usr/lib/x86_64-linux-gnu/libpthread.so \
        /usr/lib/x86_64-linux-gnu/libQtOpenGL.so \
        /usr/lib/x86_64-linux-gnu/libboost_system.so \
        /usr/lib/x86_64-linux-gnu/libboost_filesystem.so \
        /usr/lib/x86_64-linux-gnu/libboost_thread.so \
        /usr/lib/libOpenNI.so

LIBS +=/usr/local/lib/libpcl_common.so \
        /usr/local/lib/libpcl_filters.so \
        /usr/local/lib/libpcl_io_ply.so \
        /usr/local/lib/libpcl_io.so \
        /usr/local/lib/libpcl_kdtree.so \
        /usr/local/lib/libpcl_octree.so \
        /usr/local/lib/libpcl_outofcore.so \
        /usr/local/lib/libpcl_people.so \
        /usr/local/lib/libpcl_registration.so \
        /usr/local/lib/libpcl_sample_consensus.so \
        /usr/local/lib/libpcl_search.so \
        /usr/local/lib/libpcl_segmentation.so \
        /usr/local/lib/libpcl_stereo.so \
        /usr/local/lib/libpcl_surface.so \
        /usr/local/lib/libpcl_visualization.so

LIBS += -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lrt -lpthread -lm -ldl


HEADERS += \
#     SLIC_cuda.h \
    SLIC.h \
    SPixels.h \
    Depth_canny.h \
    segment.h \
    Depth_Gmm.h

SOURCES += \
    main.cpp \
#     SLIC_cuda.cpp \
    SLIC.cpp \
    SPixels.cpp \
    segment.cpp \
    Depth_Gmm.cpp

# OTHER_FILES += SLIC_cuda.cu
# CUDA_SOURCES += SLIC_cuda.cu
