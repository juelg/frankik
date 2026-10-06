# Pinocchio from the `pin` PyPI wheel: <cmeel prefix>/{include,lib}.
# Provides pinocchio::pinocchio, pinocchio::parsers and pinocchio::all.
if(pinocchio_FOUND)
    return()
endif()

execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m cmeel cmake
    OUTPUT_VARIABLE _pinocchio_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT EXISTS "${_pinocchio_prefix}/include/pinocchio")
    cmake_path(APPEND Python3_SITELIB cmeel.prefix OUTPUT_VARIABLE _pinocchio_prefix)
endif()
if(APPLE)
    set(_lib_suffix dylib)
else()
    set(_lib_suffix "so.*")
endif()
file(GLOB _pinocchio_default_lib "${_pinocchio_prefix}/lib/libpinocchio_default.${_lib_suffix}")
file(GLOB _pinocchio_parsers_lib "${_pinocchio_prefix}/lib/libpinocchio_parsers.${_lib_suffix}")
list(SORT _pinocchio_default_lib ORDER DESCENDING)
list(SORT _pinocchio_parsers_lib ORDER DESCENDING)

if(NOT EXISTS "${_pinocchio_prefix}/include/pinocchio" OR NOT _pinocchio_default_lib OR NOT _pinocchio_parsers_lib)
    set(pinocchio_FOUND FALSE)
    if(pinocchio_FIND_REQUIRED)
        message(FATAL_ERROR "pinocchio not found, run `pip install pin` for ${Python3_EXECUTABLE}")
    endif()
    return()
endif()

list(GET _pinocchio_default_lib 0 _pinocchio_default_lib)
list(GET _pinocchio_parsers_lib 0 _pinocchio_parsers_lib)
add_library(pinocchio::pinocchio SHARED IMPORTED)
set_target_properties(pinocchio::pinocchio PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_pinocchio_prefix}/include"
    IMPORTED_LOCATION "${_pinocchio_default_lib}"
)
add_library(pinocchio::parsers SHARED IMPORTED)
set_target_properties(pinocchio::parsers PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_pinocchio_prefix}/include"
    IMPORTED_LOCATION "${_pinocchio_parsers_lib}"
)
add_library(pinocchio::all INTERFACE IMPORTED)
set_target_properties(pinocchio::all PROPERTIES INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio;pinocchio::parsers")
set(pinocchio_FOUND TRUE)
message(STATUS "Found pinocchio in ${_pinocchio_prefix}")
