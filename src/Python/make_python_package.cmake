# Clean up directory
file(REMOVE_RECURSE ${PYTHON_PACKAGE_DST_DIR})
file(MAKE_DIRECTORY ${PYTHON_PACKAGE_DST_DIR}/open3d_keypoints)

# Create python pacakge. It contains: 1) Pure-python code and misc files, copied
# from src/Python/package 2) The compiled python-C++ module, i.e.
# open3d_keypoints.so (or the equivalents) Optionally other modules e.g.
# open3d_keypoints_tf_ops.so may be included. 3) Configured files and supporting
# files

# 1) Pure-python code and misc files, copied from src/Python/package
file(COPY
     ${PYTHON_PACKAGE_SRC_DIR}/
     DESTINATION
     ${PYTHON_PACKAGE_DST_DIR}
     # Excludes ${PYTHON_PACKAGE_SRC_DIR}/open3d_pybind/*
     PATTERN "open3d_pybind" EXCLUDE)

# 2) The compiled python-C++ module, i.e. open3d_keypoints.so (or the
# equivalents) Optionally other modules e.g. open3d_keypoints_tf_ops.so may be
# included.
list(GET COMPILED_MODULE_PATH_LIST 0 PYTHON_COMPILED_MODULE_PATH)
get_filename_component(PYTHON_COMPILED_MODULE_NAME
                       ${PYTHON_COMPILED_MODULE_PATH} NAME)
file(COPY
     ${COMPILED_MODULE_PATH_LIST}
     DESTINATION
     ${PYTHON_PACKAGE_DST_DIR}/open3d_keypoints)

configure_file("${PYTHON_PACKAGE_SRC_DIR}/setup.py"
               "${PYTHON_PACKAGE_DST_DIR}/setup.py")
configure_file("${PYTHON_PACKAGE_SRC_DIR}/open3d_keypoints/__init__.py"
               "${PYTHON_PACKAGE_DST_DIR}/open3d_keypoints/__init__.py")
