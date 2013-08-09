#include <Python.h>
#include <numpy/arrayobject.h>
#include <snark/point_cloud/detect_change.h>


static PyObject*
detect_change_cfunc(PyObject *dummy, PyObject *args) 
{
    PyObject *arg1=NULL, *arg2=NULL, *out=NULL;
    PyObject *arr1=NULL, *arr2=NULL, *oarr=NULL;

    if (!PyArg_ParseTuple(args, "OOO!", &arg1, &arg2,
        &PyArray_Type, &out)) return NULL;

    arr1 = PyArray_FROM_OTF(arg1, NPY_DOUBLE, NPY_IN_ARRAY);
    if (arr1 == NULL) return NULL;
    arr2 = PyArray_FROM_OTF(arg2, NPY_DOUBLE, NPY_IN_ARRAY);
    if (arr2 == NULL) return NULL; //goto fail;
    oarr = PyArray_FROM_OTF(out, NPY_DOUBLE, NPY_INOUT_ARRAY);
    if (oarr == NULL) return NULL; //goto fail;

    /* code that makes use of arguments */
    float threshold = 0.0157;
    float range_threshold = 0.15;
    typedef snark::voxel_map<cell, 2 > grid_t;
    snark::voxel_map< int, 2 >::point_type resolution;
    resolution = grid_t::point_type( threshold, threshold );
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
                double bearing = p.bearing() + threshold * i;
                if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                double elevation = p.elevation() + threshold * j;
                grid_t::iterator it = grid.touch_at( grid_t::point_type( bearing, elevation ) );
                it->second.add( entry ); //it->second.add_to_grid( entry );
            }
        }
    }

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
            const cell::entry* q = it->second.trace( p, threshold, range_threshold );
            if( !q ) { change = false; }
        }

        *(bool *)PyArray_GETPTR1(oarr, pt_idx) = change;
    }

    Py_DECREF(arr1);
    Py_DECREF(arr2);
    Py_DECREF(oarr);
    Py_INCREF(Py_None);
    return Py_None;

 fail:
    Py_XDECREF(arr1);
    Py_XDECREF(arr2);
    PyArray_XDECREF_ERR((PyArrayObject*)oarr);
    return NULL; 
}


static PyMethodDef mymethods[] = {
    { "detect_change", detect_change_cfunc, METH_VARARGS, "Ya"},
    {NULL, NULL, 0, NULL} /* Sentinel */
};


PyMODINIT_FUNC
initdetect_change(void) {
    (void)Py_InitModule("detect_change", mymethods);
    import_array();
}


