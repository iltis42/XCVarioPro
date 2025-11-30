
file(GLOB SUBCOMM_SRCS "*.cpp")

target_sources(${COMPONENT_LIB} PRIVATE ${SUBCOMM_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
