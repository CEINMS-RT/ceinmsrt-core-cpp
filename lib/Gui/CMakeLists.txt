qt5_wrap_ui(UI_HEADER mainwindow.ui)

add_subdirectory(QCustomPlot)

add_library(opengl_util
        myGLWidget.cpp
        opengl/Mesh.cpp
        opengl/RenderedMesh.cpp
        ${UI_HEADER})
target_link_libraries(opengl_util
        qcustomplot
        GLEW::GLEW
        OpenGL::GL
        OpenGL::GLU
        Qt5::OpenGL
        Qt5::PrintSupport
        Boost::thread
        osimTools)
		
target_compile_features(opengl_util PUBLIC cxx_std_17)
target_compile_definitions(opengl_util PUBLIC -D_HAS_STD_BYTE=0)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		target_link_libraries(opengl_util stdc++fs)
endif()

add_library(Gui
        mainwindow.cpp
        ${UI_HEADER})
target_link_libraries(Gui PRIVATE
        opengl_util
        qcustomplot)
target_link_libraries(Gui PUBLIC
        GLEW::GLEW)

add_library(GuiCal STATIC
        mainwindowCal.cpp
        CalManager.cpp
        ${UI_HEADER})
target_link_libraries(GuiCal PRIVATE
        opengl_util
        SyncToolsCal
        XSD::XSD
        Qt5::Widgets)

add_library(GuiViz STATIC
        mainwindowViz.cpp
        ../Producers/SyncTools.cpp
        ../Producers/Semaphore.cpp
        QCustomPlotMI.cpp
        ${UI_HEADER})
target_link_libraries(GuiViz PRIVATE
        opengl_util)

add_library(GuiViz2File STATIC
        mainwindowViz2File.cpp
        ../Producers/SyncTools.cpp
        ../Producers/Semaphore.cpp
        ${UI_HEADER})
target_link_libraries(GuiViz2File PRIVATE
        opengl_util)
