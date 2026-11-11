include(FetchContent)

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.13.6
)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(core_bridge
    core_bridge.cpp
    urm_bindings.cpp
    kin_bindings.cpp
)

target_link_libraries(core_bridge PRIVATE core_lib Eigen3::Eigen)

target_compile_features(core_bridge PRIVATE cxx_std_20)

# Copy the built .pyd to both bindings/ and project root for python imports
add_custom_command(TARGET core_bridge POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:core_bridge>
        ${CMAKE_SOURCE_DIR}/bindings/$<TARGET_FILE_NAME:core_bridge>
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:core_bridge>
        ${CMAKE_SOURCE_DIR}/$<TARGET_FILE_NAME:core_bridge>
    COMMENT "Copying core_bridge to bindings/ and project root"
)
