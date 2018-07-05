TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11
INCLUDEPATH += /usr/local/include/pcl-1.8

#INCLUDEPATH += /usr/local/include/opencv2

INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /usr/include/ni

INCLUDEPATH += /usr/include/vtk-5.8

INCLUDEPATH += /usr/include/boost

INCLUDEPATH += /usr/include/triclops/

INCLUDEPATH += /usr/include/flycapture

INCLUDEPATH += /home/master-wang/caffe-ssd/include

INCLUDEPATH += /home/master-wang/caffe-ssd/build/include


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

LIBS += /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_flann.so \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_imgproc.so \

LIBS += /usr/lib/libvtkalglib.so \
        /usr/lib/libvtkCharts.so \
        /usr/lib/libvtkCommon.so \
        /usr/lib/libvtkDICOMParser.so \
        /usr/lib/libvtkexoIIc.so \
        /usr/lib/libvtkFiltering.so \
        /usr/lib/libvtkftgl.so \
        /usr/lib/libvtkGenericFiltering.so \
        /usr/lib/libvtkGeovis.so \
        /usr/lib/libvtkGraphics.so \
        /usr/lib/libvtkHybrid.so \
        /usr/lib/libvtkImaging.so \
        /usr/lib/libvtkInfovis.so \
        /usr/lib/libvtkIO.so \
        /usr/lib/libvtkmetaio.so \
        /usr/lib/libvtkParallel.so \
        /usr/lib/libvtkproj4.so \
        /usr/lib/libvtkQtChart.so \
        /usr/lib/libvtkRendering.so \
        /usr/lib/libvtksys.so \
        /usr/lib/libvtkverdict.so \
        /usr/lib/libvtkViews.so \
        /usr/lib/libvtkVolumeRendering.so \
        /usr/lib/libvtkWidgets.so

LIBS +=  /usr/lib/x86_64-linux-gnu/libboost_atomic.so \
        /usr/lib/x86_64-linux-gnu/libboost_chrono.so \
        /usr/lib/x86_64-linux-gnu/libboost_context.so \
        /usr/lib/x86_64-linux-gnu/libboost_date_time.so \
        /usr/lib/x86_64-linux-gnu/libboost_filesystem.so \
        /usr/lib/x86_64-linux-gnu/libboost_graph_parallel.so \
        /usr/lib/x86_64-linux-gnu/libboost_graph.so \
        /usr/lib/x86_64-linux-gnu/libboost_iostreams.so \
        /usr/lib/x86_64-linux-gnu/libboost_locale.so \
        /usr/lib/x86_64-linux-gnu/libboost_log_setup.so \
        /usr/lib/x86_64-linux-gnu/libboost_log.so \
        /usr/lib/x86_64-linux-gnu/libboost_math_c99f.so \
        /usr/lib/x86_64-linux-gnu/libboost_math_c99l.so \
        /usr/lib/x86_64-linux-gnu/libboost_math_c99.so \
        /usr/lib/x86_64-linux-gnu/libboost_math_tr1f.so \
        /usr/lib/x86_64-linux-gnu/libboost_math_tr1l.so \
        /usr/lib/x86_64-linux-gnu/libboost_math_tr1.so \
        /usr/lib/x86_64-linux-gnu/libboost_mpi_python-py27.so \
        /usr/lib/x86_64-linux-gnu/libboost_mpi_python-py34.so \
        /usr/lib/x86_64-linux-gnu/libboost_mpi_python.so \
        /usr/lib/x86_64-linux-gnu/libboost_mpi.so \
        /usr/lib/x86_64-linux-gnu/libboost_prg_exec_monitor.so \
        /usr/lib/x86_64-linux-gnu/libboost_program_options.so \
        /usr/lib/x86_64-linux-gnu/libboost_python-py27.so \
        /usr/lib/x86_64-linux-gnu/libboost_python-py34.so \
        /usr/lib/x86_64-linux-gnu/libboost_python.so \
        /usr/lib/x86_64-linux-gnu/libboost_random.so \
        /usr/lib/x86_64-linux-gnu/libboost_regex.so \
        /usr/lib/x86_64-linux-gnu/libboost_serialization.so \
        /usr/lib/x86_64-linux-gnu/libboost_signals.so \
        /usr/lib/x86_64-linux-gnu/libboost_system.so \
        /usr/lib/x86_64-linux-gnu/libboost_thread.so \
        /usr/lib/x86_64-linux-gnu/libboost_timer.so \
        /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so \
        /usr/lib/x86_64-linux-gnu/libboost_wave.so \
        /usr/lib/x86_64-linux-gnu/libboost_wserialization.so

LIBS += -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lrt -lpthread -lm -ldl


SOURCES += \
    main.cpp
