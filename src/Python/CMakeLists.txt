find_package(PythonExecutable REQUIRED) # invokes the module in 3rdparty/CMake
find_package(pybind11 REQUIRED)
set(PYPI_PACKAGE_NAME "open3d_keypoints")
set(PACKAGE_NAME open3d_pybind)

# We need to get python version to configure some meta files
execute_process(
  COMMAND
    ${PYTHON_EXECUTABLE} -c
    "import sys; print('%d.%d' % (sys.version_info.major, sys.version_info.minor))"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Using Python version: ${PYTHON_VERSION}")

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
                        "import sys; print('%d' % (sys.version_info.major,))"
                OUTPUT_VARIABLE PYTHON_VERSION_MAJOR
                OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Using Python version major: ${PYTHON_VERSION_MAJOR}")

file(GLOB_RECURSE PY_ALL_SOURCE_FILES "open3d_pybind/*.cpp")

pybind11_add_module(${PACKAGE_NAME} ${PY_ALL_HEADER_FILES}
                    ${PY_ALL_SOURCE_FILES})

target_include_directories(${PACKAGE_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PACKAGE_NAME}
                      PRIVATE ${CMAKE_PROJECT_NAME} ${Open3D_LIBRARIES})

# At `make`: open3d.so (or the equivalents) will be created at
# PYTHON_COMPILED_MODULE_DIR. The default locaiton is `build/lib/Python`
set(PYTHON_COMPILED_MODULE_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/Python")
set_target_properties(${PACKAGE_NAME}
                      PROPERTIES FOLDER
                                 "Python"
                                 LIBRARY_OUTPUT_DIRECTORY
                                 "${PYTHON_COMPILED_MODULE_DIR}"
                                 ARCHIVE_OUTPUT_DIRECTORY
                                 "${PYTHON_COMPILED_MODULE_DIR}")

# Use `make python-package` to create the python package in the build directory
# The python package will be created at PYTHON_PACKAGE_DIR. It contains: 1)
# Pure-python code and misc files, copied from src/Python/package 2) The
# compiled python-C++ module, i.e. open3d.so (or the equivalents) 3) Configured
# files and supporting files Note: `make python-package` clears
# PYTHON_COMPILED_MODULE_DIR first every time
set(PYTHON_PACKAGE_DST_DIR "${CMAKE_BINARY_DIR}/lib/python_package")
message(STATUS "PYPI_PACKAGE_NAME: ${PYPI_PACKAGE_NAME}")

# add the open3d python module first
set(COMPILED_MODULE_PATH_LIST $<TARGET_FILE:${PACKAGE_NAME}>)

add_custom_target(
  python-package
  COMMAND ${CMAKE_COMMAND} -DPYTHON_PACKAGE_SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
          -DPYTHON_PACKAGE_DST_DIR=${PYTHON_PACKAGE_DST_DIR}
          -DPYTHON_VERSION=${PYTHON_VERSION}
          "-DCOMPILED_MODULE_PATH_LIST=${COMPILED_MODULE_PATH_LIST}"
          -DENABLE_JUPYTER=${ENABLE_JUPYTER}
          -DBUILD_TENSORFLOW_OPS=${BUILD_TENSORFLOW_OPS}
          -DPROJECT_EMAIL=${PROJECT_EMAIL} -DPROJECT_HOME=${PROJECT_HOME}
          -DPROJECT_DOCS=${PROJECT_DOCS} -DPROJECT_CODE=${PROJECT_CODE}
          -DPROJECT_ISSUES=${PROJECT_ISSUES}
          -DPROJECT_VERSION=${PROJECT_VERSION}
          -DPROJECT_VERSION_THREE_NUMBER=${PROJECT_VERSION_THREE_NUMBER}
          -DPYPI_PACKAGE_NAME=${PYPI_PACKAGE_NAME} -P
          ${CMAKE_CURRENT_SOURCE_DIR}/make_python_package.cmake
  VERBATIM)

# Use `make pip-package` to create the pip package in the build directory
add_custom_target(
  pip-package
  COMMAND ${PYTHON_EXECUTABLE} setup.py bdist_wheel
          --dist-dir pip_package
  COMMAND echo "pip wheel created at ${PYTHON_PACKAGE_DST_DIR}/pip_package"
  WORKING_DIRECTORY ${PYTHON_PACKAGE_DST_DIR}
  DEPENDS python-package)

# Use `make install-pip-package` to install pip wheel package to the current
# python environment.
add_custom_target(
  install-pip-package
  COMMAND ${CMAKE_COMMAND} -DPYTHON_PACKAGE_DST_DIR=${PYTHON_PACKAGE_DST_DIR} -P
          ${CMAKE_CURRENT_SOURCE_DIR}/make_install_pip_package.cmake
  DEPENDS pip-package)

# FOR DEBUGGING ONLY Use `make install-python-package` to build and install
# python package in the current python environment. This is substantially faster
# than `make install-pip-package`. However this approach does not create wheel
# or egg files and does not take care of dependencies thus not suitable for
# deployment. Ref: https://stackoverflow.com/a/33791008/1255535
add_custom_target(install-python-package
                  COMMAND ${PYTHON_EXECUTABLE} setup.py install
                          --single-version-externally-managed
                          --root=/
                  WORKING_DIRECTORY ${PYTHON_PACKAGE_DST_DIR}
                  DEPENDS python-package)
