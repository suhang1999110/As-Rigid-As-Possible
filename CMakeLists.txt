cmake_minimum_required(VERSION 3.15)

project(as-rigid-as-possible)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC mesh.cpp include/matrix.hpp)

configure_file("${PROJECT_SOURCE_DIR}/config.h.in" "${PROJECT_BINARY_DIR}/config.h")

add_executable(main main.cpp ${SRC})
add_executable(animation animation.cpp ${SRC})
add_executable(plane plane_animation.cpp ${SRC})
add_executable(sphere sphere_animation.cpp ${SRC})

include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_BINARY_DIR}
)

target_link_libraries(main
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(main
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(animation
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(animation
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(plane
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(plane
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(sphere
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(sphere
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)
