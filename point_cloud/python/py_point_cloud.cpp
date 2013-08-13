// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author peter morton 

#include <Python.h>
#define PY_ARRAY_UNIQUE_SYMBOL _snark_point_cloud_ARRAY_API
#include <numpy/arrayobject.h>
#include <snark/point_cloud/python/py_detect_change.h>
#include <snark/point_cloud/python/py_partition.h>

const char* detect_change_doc = \
"Equivalent of points-detect-change\n" 
"\n"
"Input: reference (range, bearing, elevation) -- (N, 3) np.ndarray\n"
"       scan (range, bearing, elevation) -- (M, 3) np.ndarray\n"
"\n"
"Parameters:\n"
"    angle_threshold\n"
"    range_threshold\n";


const char* partition_doc = \
"Equivalent of points to paritions\n"
"\n"
"Inputs: points -- (N, 3) np.ndarray\n"
"\n"
"Parameters:\n"
"    resolution\n"
"    min_density\n"
"    min_points_per_voxel\n"
"    min_points_per_partition\n"
"    min_voxels_per_partition\n";


// This array defines python API entry points and their corresponding
// C implementations. 
static PyMethodDef mymethods[] = {
    { "detect_change", (PyCFunction)detect_change_cfunc, METH_VARARGS|METH_KEYWORDS, detect_change_doc},
    { "partition", (PyCFunction)partition_cfunc, METH_VARARGS|METH_KEYWORDS, partition_doc},
    {NULL, NULL, 0, NULL} /* Sentinel */
};

// Standard init func that sets up the python bindings.
PyMODINIT_FUNC
initsnark_py_point_cloud(void) {
    PyObject* m;
    m = Py_InitModule("snark_py_point_cloud", mymethods);
    PyModule_AddIntMacro(m, ID_INVALID);
    import_array();
}
