
file(GLOB SUB_SRCS "*.cpp" "element/*.cpp")

target_sources(${COMPONENT_LIB} PRIVATE ${SUB_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(${COMPONENT_LIB} PRIVATE
    -ffast-math
)