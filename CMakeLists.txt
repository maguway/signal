cmake_minimum_required(VERSION 3.20)
project(signal_gui LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/vendor)
include(FetchContent)

# ========== GLFW ==========
FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ========== ImGui ==========
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG docking GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(imgui)

# ========== ImPlot ==========
FetchContent_Declare(implot GIT_REPOSITORY https://github.com/epezent/implot.git GIT_TAG master GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(implot)

# ========== KISSFFT ==========
set(KISSFFT_TOOLS OFF CACHE BOOL "" FORCE)
set(KISSFFT_TEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(kissfft GIT_REPOSITORY https://github.com/mborgerding/kissfft.git GIT_TAG 131.1.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(kissfft)

# ========== EIGEN (БЕЗОПАСНОЕ ПОДКЛЮЧЕНИЕ) ==========
FetchContent_Declare(eigen
        GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL
)
FetchContent_Populate(eigen)
set(EIGEN_INCLUDE_DIR ${eigen_SOURCE_DIR})

# =============================================
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/vendor/imgui-src)
set(IMPLOT_DIR ${CMAKE_SOURCE_DIR}/vendor/implot-src)
set(GLFW_DIR ${CMAKE_SOURCE_DIR}/vendor/glfw-src)
set(KISSFFT_DIR ${CMAKE_SOURCE_DIR}/vendor/kissfft-src)

# Проверки (разбиты на строки)
if(NOT EXISTS ${IMGUI_DIR}/imgui.h)
    message(FATAL_ERROR "imgui.h not found")
endif()
if(NOT EXISTS ${IMPLOT_DIR}/implot.h)
    message(FATAL_ERROR "implot.h not found")
endif()
if(NOT EXISTS ${GLFW_DIR}/include/GLFW/glfw3.h)
    message(FATAL_ERROR "glfw3.h not found")
endif()
if(NOT EXISTS ${KISSFFT_DIR}/kiss_fft.c)
    message(FATAL_ERROR "kiss_fft.c not found")
endif()
if(NOT EXISTS ${EIGEN_INCLUDE_DIR}/Eigen/Dense)
    message(FATAL_ERROR "Eigen/Dense not found")
endif()
message(STATUS "✅ All headers found!")

# ========== СОЗДАНИЕ EXECUTABLE ==========
# ✅ ДОБАВЛЕНЫ: io/serial_reader.cpp и algorithms/doa_engine.cpp
add_executable(signal
        main.cpp
        core/signal_processor.cpp
        io/serial_reader.cpp
        algorithms/doa_engine.cpp
)

target_include_directories(signal PRIVATE
        ${CMAKE_SOURCE_DIR}              # Важно для #include "core/types.h" из любых папок
        ${CMAKE_SOURCE_DIR}/core
        ${CMAKE_SOURCE_DIR}/io
        ${CMAKE_SOURCE_DIR}/algorithms
        ${EIGEN_INCLUDE_DIR}             # Путь к Eigen
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
        ${IMPLOT_DIR}
        ${GLFW_DIR}/include
        ${KISSFFT_DIR}
)

target_sources(signal PRIVATE
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        ${IMPLOT_DIR}/implot.cpp
        ${IMPLOT_DIR}/implot_items.cpp
        ${KISSFFT_DIR}/kiss_fft.c
)

target_link_libraries(signal PRIVATE glfw opengl32)

if(WIN32)
    target_link_options(signal PRIVATE -static -static-libgcc -static-libstdc++)
endif()