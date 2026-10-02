# Locate the Pinocchio C++ library shipped inside the `pin` PyPI wheel (cmeel layout).
#
# The wheel installs headers and shared libraries into
#   <site-packages>/cmeel.prefix/{include,lib}
# This module creates the imported targets
#   pinocchio::pinocchio  (libpinocchio_default)
#   pinocchio::parsers    (libpinocchio_parsers, URDF/MJCF parsers)
#   pinocchio::all        (interface target linking both)
# and sets pinocchio_FOUND, pinocchio_VERSION, pinocchio_INCLUDE_DIRS, pinocchio_LIBRARY_DIR.
#
# Pinocchio is deliberately not consumed through its own pinocchioConfig.cmake because that
# config pulls in Eigen3, eigenpy, hpp-fcl, urdfdom, ... config files which are not all shipped
# in the wheel. frankik only needs the headers plus the two core libraries.

if(pinocchio_FOUND)
    return()
endif()

if(NOT Python3_FOUND)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
endif()

set(_pinocchio_prefix "")
# Preferred: ask cmeel for its prefix (works for site-packages, user installs and venvs).
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m cmeel cmake
    OUTPUT_VARIABLE _cmeel_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _cmeel_result
)
if(_cmeel_result EQUAL 0 AND EXISTS "${_cmeel_prefix}/include/pinocchio")
    set(_pinocchio_prefix "${_cmeel_prefix}")
endif()
# Fallback: the conventional location next to the python packages.
if(_pinocchio_prefix STREQUAL "" AND Python3_SITELIB)
    cmake_path(APPEND Python3_SITELIB cmeel.prefix OUTPUT_VARIABLE _sitelib_prefix)
    if(EXISTS "${_sitelib_prefix}/include/pinocchio")
        set(_pinocchio_prefix "${_sitelib_prefix}")
    endif()
endif()

if(_pinocchio_prefix STREQUAL "")
    set(pinocchio_FOUND FALSE)
    if(pinocchio_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find pinocchio. Install it with `pip install pin` (the cmeel wheel) "
                            "into the Python environment used for the build (${Python3_EXECUTABLE}).")
    endif()
    return()
endif()

set(pinocchio_INCLUDE_DIRS "${_pinocchio_prefix}/include")
set(pinocchio_LIBRARY_DIR "${_pinocchio_prefix}/lib")

set(_pinocchio_default_globs)
set(_pinocchio_parsers_globs)
if(APPLE)
    list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_default*.dylib")
    list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_parsers*.dylib")
elseif(WIN32)
    list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/pinocchio_default*.lib" "${pinocchio_LIBRARY_DIR}/libpinocchio_default*.lib")
    list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/pinocchio_parsers*.lib" "${pinocchio_LIBRARY_DIR}/libpinocchio_parsers*.lib")
else()
    list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_default.so*")
    list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_parsers.so*")
endif()

file(GLOB _pinocchio_default_paths LIST_DIRECTORIES FALSE ${_pinocchio_default_globs})
file(GLOB _pinocchio_parsers_paths LIST_DIRECTORIES FALSE ${_pinocchio_parsers_globs})
list(LENGTH _pinocchio_default_paths _n_default)
list(LENGTH _pinocchio_parsers_paths _n_parsers)
if(_n_default EQUAL 0 OR _n_parsers EQUAL 0)
    set(pinocchio_FOUND FALSE)
    if(pinocchio_FIND_REQUIRED)
        message(FATAL_ERROR "Found pinocchio headers in ${pinocchio_INCLUDE_DIRS} but no libraries. "
                            "Searched: ${_pinocchio_default_globs} ${_pinocchio_parsers_globs}")
    endif()
    return()
endif()
# Prefer the fully versioned file name (libpinocchio_default.so.3.7.0) so the SONAME is recorded.
list(SORT _pinocchio_default_paths ORDER DESCENDING)
list(SORT _pinocchio_parsers_paths ORDER DESCENDING)
list(GET _pinocchio_default_paths 0 _pinocchio_default_lib)
list(GET _pinocchio_parsers_paths 0 _pinocchio_parsers_lib)

# Version from the python distribution metadata
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import importlib.metadata as m; print(m.version('pin'))"
    OUTPUT_VARIABLE pinocchio_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

add_library(pinocchio::pinocchio SHARED IMPORTED)
set_target_properties(pinocchio::pinocchio PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${pinocchio_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${_pinocchio_default_lib}"
)
if(WIN32)
    set_target_properties(pinocchio::pinocchio PROPERTIES IMPORTED_IMPLIB "${_pinocchio_default_lib}")
endif()

add_library(pinocchio::parsers SHARED IMPORTED)
set_target_properties(pinocchio::parsers PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${pinocchio_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${_pinocchio_parsers_lib}"
)
if(WIN32)
    set_target_properties(pinocchio::parsers PROPERTIES IMPORTED_IMPLIB "${_pinocchio_parsers_lib}")
endif()

add_library(pinocchio::all INTERFACE IMPORTED)
set_target_properties(pinocchio::all PROPERTIES
    INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio;pinocchio::parsers"
)

set(pinocchio_FOUND TRUE)
message(STATUS "Found pinocchio ${pinocchio_VERSION} in ${_pinocchio_prefix}")
