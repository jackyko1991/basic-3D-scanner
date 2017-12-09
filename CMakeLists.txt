cmake_minimum_required(VERSION 2.8.9)

project(ScannerProject)

FIND_PACKAGE(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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
  qt5_use_modules(ScannerProject Core Gui Concurrent)
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