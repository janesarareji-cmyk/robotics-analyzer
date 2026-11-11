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
)

target_link_libraries(core_bridge PRIVATE core_lib)

target_compile_features(core_bridge PRIVATE cxx_std_20)

# Copy the built .pyd to a known location so pytest can import it
add_custom_command(TARGET core_bridge POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:core_bridge>
        ${CMAKE_SOURCE_DIR}/bindings/$<TARGET_FILE_NAME:core_bridge>
    COMMENT "Copying core_bridge to bindings/"
)
