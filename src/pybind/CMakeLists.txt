pybind11_add_module(_core MODULE bindings.cpp)
target_link_libraries(_core PRIVATE Eigen3::Eigen)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# in pip
install(TARGETS _core DESTINATION frankik COMPONENT python_package)
