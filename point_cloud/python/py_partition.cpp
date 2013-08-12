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


PyObject* partition_cfunc(PyObject *dummy, PyObject *args, PyObject *kwds) 
{
    PyObject *arg1=NULL;
    PyObject *arr1=NULL;

    double resolution = 0.2;
    double min_density = 0.0;
    int min_points_per_voxel = 1;
    int min_points_per_partition = 1;
    int min_voxels_per_partition = 1;

    static char *kwlist[] = {
        "points",
        "resolution",
        "min_density",
        "min_points_per_voxel",
        "min_points_per_partition",
        "min_voxels_per_partition",
        NULL
    };

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "O!|ddiii", 
            kwlist, &PyArray_Type, &arg1,
            &resolution, &min_density, &min_points_per_voxel,
            &min_points_per_partition, &min_voxels_per_partition)) return NULL;

    arr1 = PyArray_FROM_OTF(arg1, NPY_DOUBLE, NPY_IN_ARRAY);
    if (arr1 == NULL) return NULL;

    boost::optional< snark::math::closed_interval< double, 3 > > extents;

    std::vector<InputPoint> points;
    points.reserve(PyArray_DIM(arr1, 0));

    // Compute extents.
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

    Eigen::Vector3d resolution3;
    resolution3 << resolution, resolution, resolution;
    snark::partition partition( *extents, resolution3, min_points_per_voxel );

    for(std::size_t i = 0; i < points.size(); ++i )
    {
        points[i].partition = &partition.insert( points[i].point );
    }

    partition.commit(min_voxels_per_partition, min_points_per_partition, 0);

    npy_intp dims[1];
    dims[0] = points.size();
    PyObject* oarr = PyArray_SimpleNew(1, dims, NPY_UINT32);

    for(std::size_t i = 0; i < points.size(); ++i )
    {
        const InputPoint& point = points[i];
        comma::uint32 id = UINT_MAX; // Not partitioned.
        if( point.partition && *point.partition ) {
            id = **point.partition;
        }
        *(comma::uint32 *)PyArray_GETPTR1(oarr, i) = id;
    }

    Py_DECREF(arr1);
    return oarr;
}
