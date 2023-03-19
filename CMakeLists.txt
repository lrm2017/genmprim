cmake_minimum_required(VERSION 3.5)

project(genmprim VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGL)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
endif()
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})

include_directories(    # 包含路径后，可以直接引入头文件名字而不需要路径了
    ${catkin_INCLUDE_DIRS}
    data
    plot3d
    include
)

set(MAIN_PATH ${PROJECT_SOURCE_DIR})    # 获取当前工程路径
configure_file(${MAIN_PATH}/data/projectpath.h.in ${MAIN_PATH}/data/project_path.h)    # 生成工程路径

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        src/utils.cpp
        plot3d/QPlot3D.cpp
        plot3d/QPlot3D.h
        mainwindow.h
        mainwindow.ui
)


add_executable(genmprim
    ${PROJECT_SOURCES}
)

ament_target_dependencies( ${PROJECT_NAME}
  rclcpp
)

target_link_libraries(${PROJECT_NAME}
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    MPs
    -lGL
  )


add_subdirectory(spiral_constraint)

set_target_properties(genmprim PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS genmprim
  DESTINATION lib/${PROJECT_NAME})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(genmprim)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
