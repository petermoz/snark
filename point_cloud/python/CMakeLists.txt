# Build Python extension module

SET(target "snark_py_point_cloud")

# Python headers
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

# Python executable

# Python site packages
# http://stackoverflow.com/a/14796658
EXECUTE_PROCESS(
    COMMAND python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" 
    OUTPUT_VARIABLE PYTHON_DIST_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Find NumPy by asking python where it is
FIND_PACKAGE(PythonInterp 2.7 REQUIRED)
EXEC_PROGRAM("${PYTHON_EXECUTABLE}"
    ARGS "-c 'import numpy; print(numpy.get_include())'"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    RETURN_VALUE NUMPY_NOT_FOUND)
SET(NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_DIR} CACHE PATH "Numpy include path")
INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

FILE(GLOB source *.cpp)
FILE(GLOB includes *.h)
ADD_LIBRARY(${target} ${source})

TARGET_LINK_LIBRARIES(${target} 
    ${PYTHON_LIBARY}
    ${snark_ALL_LIBRARIES} 
) 

SET_TARGET_PROPERTIES(${target} PROPERTIES PREFIX "")

INSTALL(
    TARGETS ${target} DESTINATION ${PYTHON_DIST_PACKAGES}
)
