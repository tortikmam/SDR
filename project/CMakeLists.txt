cmake_minimum_required(VERSION 3.16)
project(PlutoSDR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(SoapySDR REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/../libs/imgui)
set(IMPLOT_DIR ${CMAKE_SOURCE_DIR}/../libs/implot)

add_executable(main
    main.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

target_include_directories(main PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
    ${SoapySDR_INCLUDE_DIRS}
)

target_link_libraries(main PRIVATE
    ${SoapySDR_LIBRARIES}
    glfw OpenGL::GL Threads::Threads m
)