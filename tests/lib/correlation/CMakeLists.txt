
project(CorrelationLib)

add_library(${PROJECT_NAME} correlation.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})

# target_link_libraries(${PROJECT_NAME}
# 	Boost::unit_test_framework)