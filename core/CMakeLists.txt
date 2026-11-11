add_library(core_lib STATIC
    src/placeholder.cpp
    urm/robot_model.cpp
    kin/fk.cpp
    kin/compiled_model.cpp
)

target_include_directories(core_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(core_lib PUBLIC cxx_std_20)

# Link Eigen and nlohmann_json (header-only)
target_link_libraries(core_lib PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
