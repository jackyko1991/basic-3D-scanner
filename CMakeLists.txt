cmake_minimum_required(VERSION 2.8.9)

project(ScannerProject)

FIND_PACKAGE(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5Widgets REQUIRED QUIET)
else()
  find_package(Qt4 REQUIRED)
  include(${QT_USE_FILE})
endif()

# Find includes in corresponding build directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

# set(qt_HEADERS mainwindow.h)
# set(qt_SOURCES main.cpp mainwindow.cpp)
# set(qt_UIS mainwindow.ui)
# set(qt_RESOURCES resources.qrc)

# ##optimization
# add_definitions(-O3 -Wno-deprecated -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
# #add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
# add_definitions (-O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)

file(GLOB UI_FILES *.ui)
file(GLOB QT_WRAP *.h)
file(GLOB C_FILES *.c)
file(GLOB CXX_FILES *.cxx *.cpp)
file(GLOB QRC_RESOURCES *.qrc)

if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  qt5_wrap_ui(UISrcs ${UI_FILES} )
  qt5_add_resources(QT_RESOURCES ${QRC_RESOURCES})
  # CMAKE_AUTOMOC in ON so the MOC headers will be automatically wrapped.
  add_executable(ScannerProject MACOSX_BUNDLE
    ${CXX_FILES} ${C_FILES} ${UISrcs} ${QT_WRAP} ${QT_RESOURCES})
  qt5_use_modules(ScannerProject Core Gui)
  target_link_libraries(ScannerProject ${VTK_LIBRARIES} ${PCL_LIBRARIES})
else()
  QT4_WRAP_UI(UISrcs ${UI_FILES})
  QT4_WRAP_CPP(MOCSrcs ${QT_WRAP})
  qt4_add_resources(QT_RESOURCES ${QRC_RESOURCES})
  add_executable(ScannerProject MACOSX_BUNDLE 
  	${CXX_FILES} ${C_FILES} ${UISrcs} ${MOCSrcs} ${QT_RESOURCES})
 
  if(VTK_LIBRARIES)
    if(${VTK_VERSION} VERSION_LESS "6")
      target_link_libraries(ScannerProject ${VTK_LIBRARIES} QVTK ${PCL_LIBRARIES})
    else()
      target_link_libraries(ScannerProject ${VTK_LIBRARIES} ${PCL_LIBRARIES})
    endif()
  else()
    target_link_libraries(ScannerProject vtkHybrid QVTK vtkViews ${QT_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()

# qt4_wrap_cpp(myproject_MOC ${qt_HEADERS})
# qt4_wrap_ui(qt_UIS_H ${qt_UIS})



# add_executable(ScannerProject ${qt_SOURCES}
#     ${myproject_MOC}
#     ${qt_RESOURCES_QRC}
#     ${qt_UIS_H} rs232.c motordriver.cpp)

# target_link_libraries(ScannerProject ${QT_LIBRARIES} QVTK vtkRendering ${PCL_LIBRARIES})
