# Analytical Franka solver (no external runtime dependencies)
pybind11_add_module(_core MODULE bindings.cpp)
target_link_libraries(_core PRIVATE Eigen3::Eigen)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})
install(TARGETS _core DESTINATION frankik COMPONENT python_package)

# Numerical solver backed by Pinocchio (taken from the `pin` PyPI wheel)
if(NOT FRANKIK_WITH_PINOCCHIO STREQUAL "OFF")
    if(FRANKIK_WITH_PINOCCHIO STREQUAL "AUTO")
        find_package(pinocchio)
    else()
        find_package(pinocchio REQUIRED)
    endif()
endif()

if(pinocchio_FOUND)
    if(APPLE)
        set(_rpath_origin "@loader_path")
    else()
        set(_rpath_origin "$ORIGIN")
    endif()
    pybind11_add_module(_pin MODULE pin_bindings.cpp)
    target_link_libraries(_pin PRIVATE Eigen3::Eigen pinocchio::all)
    target_compile_definitions(_pin PRIVATE VERSION_INFO=${PROJECT_VERSION})
    # The pinocchio shared libraries live next to the python packages in
    # <site-packages>/cmeel.prefix/lib, _pin is installed to <site-packages>/frankik.
    set_target_properties(_pin PROPERTIES
        INSTALL_RPATH "${_rpath_origin}/../cmeel.prefix/lib"
    )
    install(TARGETS _pin DESTINATION frankik COMPONENT python_package)
    message(STATUS "frankik: building the Pinocchio numerical solver (_pin)")
else()
    message(STATUS "frankik: pinocchio not found, building without the numerical solver (_pin)")
endif()
