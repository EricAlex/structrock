cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR)

file(COPY qcustomplot.h DESTINATION ${CMAKE_BINARY_DIR})

project(structrock)
find_package(Qt4 REQUIRED)
find_package(PCL 1.6.0 REQUIRED)

if(${VTK_VERSION} VERSION_GREATER "6")
find_package(VTK COMPONENTS
vtkGUISupportQt
vtkRenderingQt
vtkViewsQt
)
else()
find_package(VTK REQUIRED)
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB project_SOURCES . *.cpp)

file(GLOB project_HEADERS . *.h)

file(GLOB project_FORMS . *.ui)

QT4_WRAP_CPP(project_HEADERS_MOC ${project_HEADERS})
QT4_WRAP_UI(project_FORMS_HEADERS ${project_FORMS})

INCLUDE(${QT_USE_FILE})
INCLUDE(${VTK_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

set(myApp_ICON ${CMAKE_CURRENT_SOURCE_DIR}/icon.icns)

if(WIN32)
set(GUI_TYPE WIN32)
endif(WIN32)
if(APPLE)
set(GUI_TYPE MACOSX_BUNDLE)
set(MACOSX_BUNDLE_ICON_FILE icon.icns)
SET_SOURCE_FILES_PROPERTIES(${myApp_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
endif(APPLE)

ADD_EXECUTABLE(structrock ${GUI_TYPE} ${myApp_ICON} ${project_SOURCES} ${project_FORMS_HEADERS} ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES(structrock ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

#INSTALL(SCRIPT bundle.cmake)