#include <Python.h>
#define PY_ARRAY_UNIQUE_SYMBOL _snark_point_cloud_ARRAY_API
#define NO_IMPORT_ARRAY
#include <numpy/arrayobject.h>
#include <snark/point_cloud/detect_change.h>
#include <snark/point_cloud/python/py_detect_change.h>


PyObject* detect_change_cfunc(PyObject *dummy, PyObject *args, PyObject *kwds)
{
    PyObject *arg1=NULL, *arg2=NULL;
    PyObject *arr1=NULL, *arr2=NULL;

    double angle_threshold = 0.0157;
    double range_threshold = 0.15;

    static char *kwlist[] = {
        "reference",
        "scan",
        "angle_threshold",
        "range_threshold",
        NULL
    };

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "O!O!|dd", kwlist,
                &PyArray_Type, &arg1, &PyArray_Type, &arg2,
                &angle_threshold, &range_threshold)) return NULL;

    arr1 = PyArray_FROM_OTF(arg1, NPY_DOUBLE, NPY_IN_ARRAY);
    if (arr1 == NULL) return NULL;
    arr2 = PyArray_FROM_OTF(arg2, NPY_DOUBLE, NPY_IN_ARRAY);
    if (arr2 == NULL) { Py_XDECREF(arr1); return NULL; }

    /* code that makes use of arguments */
    typedef snark::voxel_map<cell, 2 > grid_t;
    snark::voxel_map< int, 2 >::point_type resolution;
    resolution = grid_t::point_type( angle_threshold, angle_threshold );
    grid_t grid( resolution );

    // Load reference scan.
    comma::uint64 index = 0;
    for (int pt_idx = 0; pt_idx < PyArray_DIM(arr1, 0); pt_idx++) {
        point_t p( *(double *)PyArray_GETPTR2(arr1, pt_idx, 0),
                   *(double *)PyArray_GETPTR2(arr1, pt_idx, 1),
                   *(double *)PyArray_GETPTR2(arr1, pt_idx, 2));
        cell::entry entry(p, index);
        for( int i = -1; i < 2; ++i )
        {
            for( int j = -1; j < 2; ++j )
            {
                double bearing = p.bearing() + angle_threshold * i;
                if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                double elevation = p.elevation() + angle_threshold * j;
                grid_t::iterator it = grid.touch_at( grid_t::point_type( bearing, elevation ) );
                it->second.add( entry ); //it->second.add_to_grid( entry );
            }
        }
    }

    // Create output array.
    npy_intp dims[1];
    dims[0] = PyArray_DIM(arr2, 0);
    PyObject* oarr = PyArray_SimpleNew(1, dims, NPY_BOOL);

    // Run change detection.
    for (int pt_idx = 0; pt_idx < PyArray_DIM(arr2, 0); pt_idx++) {
        bool change = true;
        point_t p( *(double *)PyArray_GETPTR2(arr2, pt_idx, 0),
                   *(double *)PyArray_GETPTR2(arr2, pt_idx, 1),
                   *(double *)PyArray_GETPTR2(arr2, pt_idx, 2));
        grid_t::const_iterator it = grid.find( grid_t::point_type( p.bearing(), p.elevation() ) );
        if( it == grid.end() ) { 
            change = false; 
        } else {
            const cell::entry* q = it->second.trace( p, angle_threshold, range_threshold );
            if( !q ) { change = false; }
        }
        *(bool *)PyArray_GETPTR1(oarr, pt_idx) = change;
    }

    Py_DECREF(arr1);
    Py_DECREF(arr2);
    return oarr;
}

