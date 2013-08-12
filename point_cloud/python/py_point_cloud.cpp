#include <Python.h>
#define PY_ARRAY_UNIQUE_SYMBOL _snark_point_cloud_ARRAY_API
#include <numpy/arrayobject.h>
#include <snark/point_cloud/python/py_detect_change.h>
#include <snark/point_cloud/python/py_partition.h>


const char* partition_doc = \
"Equivalent of points to paritions\n"
"\n"
"Inputs: points -- (N, 3) np.ndarray\n"
"Parameters:\n"
"    resolution\n"
"    min_density\n"
"    min_points_per_voxel\n"
"    min_points_per_partition\n"
"    min_voxels_per_partition\n";


static PyMethodDef mymethods[] = {
    { "detect_change", detect_change_cfunc, METH_VARARGS, ""},
    { "partition", (PyCFunction)partition_cfunc, METH_VARARGS|METH_KEYWORDS, partition_doc},
    {NULL, NULL, 0, NULL} /* Sentinel */
};

PyMODINIT_FUNC
initsnark_py_point_cloud(void) {
    (void)Py_InitModule("snark_py_point_cloud", mymethods);
    import_array();
}
