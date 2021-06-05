cmake_minimum_required(VERSION 3.15)

project(as-rigid-as-possible)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC mesh.cpp include/matrix.hpp)

add_executable(main main.cpp ${SRC})
add_executable(test_plane test_plane.cpp ${SRC})
add_executable(test_sphere test_sphere.cpp ${SRC})


include_directories(
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(main
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(main
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(test_plane
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(test_plane
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(test_sphere
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

target_include_directories(test_sphere
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)