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
//
/// @author peter morton

#include <Python.h>
#define PY_ARRAY_UNIQUE_SYMBOL _snark_point_cloud_ARRAY_API
#define NO_IMPORT_ARRAY
#include <numpy/arrayobject.h>
#include <boost/optional.hpp>
#include <snark/point_cloud/partition.h>
#include <snark/point_cloud/python/py_partition.h>
#include <vector>


typedef Eigen::Vector3d PointType;
struct InputPoint
{
    PointType point;
    comma::uint32 id;
    const boost::optional< comma::uint32 >* partition;
    InputPoint() : partition( NULL ) {}
};

// This reimplements points-to-partitions 
PyObject* partition_cfunc(PyObject *dummy, PyObject *args, PyObject *kwds) 
{
    PyObject *arg1=NULL;
    PyObject *arr1=NULL;

    // Default values.
    double resolution = 0.2;
    double min_density = 0.0;
    int min_id = 0;
    int min_points_per_voxel = 1;
    int min_points_per_partition = 1;
    int min_voxels_per_partition = 1;

    static char *kwlist[] = {
        "points",
        "resolution",
        "min_density",
        "min_id",
        "min_points_per_voxel",
        "min_points_per_partition",
        "min_voxels_per_partition",
        NULL
    };

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "O!|ddiiii", 
            kwlist, &PyArray_Type, &arg1,
            &resolution, &min_density, &min_id, &min_points_per_voxel,
            &min_points_per_partition, &min_voxels_per_partition)) return NULL;

    arr1 = PyArray_FROM_OTF(arg1, NPY_DOUBLE, NPY_IN_ARRAY);
    if (arr1 == NULL) return NULL;

    boost::optional< snark::math::closed_interval< double, 3 > > extents;

    std::vector<InputPoint> points;
    points.reserve(PyArray_DIM(arr1, 0));

    // Compute extents of input.
    for (int pt_idx = 0; pt_idx < PyArray_DIM(arr1, 0); pt_idx++) {
        InputPoint pt;
        pt.point << *(double *)PyArray_GETPTR2(arr1, pt_idx, 0),
                    *(double *)PyArray_GETPTR2(arr1, pt_idx, 1),
                    *(double *)PyArray_GETPTR2(arr1, pt_idx, 2);
        if( !extents ) {
            extents = snark::math::closed_interval< double, 3 >( pt.point );
        } else {
            extents = extents->hull(pt.point);
        }
        points.push_back(pt);
    }

    // Insert all points into the paritioning grid.
    Eigen::Vector3d resolution3;
    resolution3 << resolution, resolution, resolution;
    snark::partition partition( *extents, resolution3, min_points_per_voxel );
    for(std::size_t i = 0; i < points.size(); ++i )
    {
        points[i].partition = &partition.insert( points[i].point );
    }

    // Create output array.
    npy_intp dims[1];
    dims[0] = points.size();
    PyObject* oarr = PyArray_SimpleNew(1, dims, NPY_UINT32);

    // Compute the output ids.
    partition.commit(min_voxels_per_partition, min_points_per_partition, min_id);
    for(std::size_t i = 0; i < points.size(); ++i )
    {
        const InputPoint& point = points[i];
        comma::uint32 id = ID_INVALID; // Not partitioned.
        if( point.partition && *point.partition ) {
            id = **point.partition;
        }
        *(comma::uint32 *)PyArray_GETPTR1(oarr, i) = id;
    }

    Py_DECREF(arr1);
    return oarr;
}
