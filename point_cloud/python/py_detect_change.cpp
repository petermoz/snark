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

#include <snark/point_cloud/python/py_detect_change.h>
#include <vector>


ChangeDetector::ChangeDetector(const bn::ndarray& ref, double angle_threshold,
                               double range_threshold)
    : grid_(grid_t::point_type(angle_threshold, angle_threshold)),
      angle_threshold_(angle_threshold), range_threshold_(range_threshold)
{
    // Check that input is as expected.
    if (ref.get_dtype() != bn::dtype::get_builtin<double>()) {
        PyErr_SetString(PyExc_TypeError, "Incorrect array data type (expected double)");
        bp::throw_error_already_set();
    }
    if(ref.get_nd() != 2 || ref.shape(1) != 3) {
        PyErr_SetString(PyExc_ValueError, "Incorrect dimensions (expected N x 3)");
        bp::throw_error_already_set();
    }

    // Compute grid.
    const int n_points = ref.shape(0);

    int row_stride = ref.strides(0) / sizeof(double);
    int col_stride = ref.strides(1) / sizeof(double);

    double *row_iter = reinterpret_cast<double *>(ref.get_data());
    for (int pt_idx = 0; pt_idx < n_points; ++pt_idx, row_iter += row_stride) {
        double *col_iter = row_iter;
        point_t p(*col_iter, 
                  *(col_iter + col_stride), 
                  *(col_iter + 2 * col_stride));
        
        cell::entry entry(p, pt_idx);
        for( int i = -1; i < 2; ++i )
        {
            for( int j = -1; j < 2; ++j )
            {
                double bearing = p.bearing() + angle_threshold * i;
                if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                double elevation = p.elevation() + angle_threshold * j;
                grid_t::iterator it = grid_.touch_at( grid_t::point_type( bearing, elevation ) );
                it->second.add( entry );
            }
        }
    }
}

bn::ndarray ChangeDetector::GetChanges(const bn::ndarray& scan, bool invert)
{
    // Check that input is as expected.
    if (scan.get_dtype() != bn::dtype::get_builtin<double>()) {
        PyErr_SetString(PyExc_TypeError, "Incorrect array data type (expected double)");
        bp::throw_error_already_set();
    }
    if(scan.get_nd() != 2 || scan.shape(1) != 3) {
        PyErr_SetString(PyExc_ValueError, "Incorrect dimensions (expected N x 3)");
        bp::throw_error_already_set();
    }

    // Create result array.
    const int n_points = scan.shape(0);
    bn::dtype dt = bn::dtype::get_builtin<bool>();
    bp::tuple shape = bp::make_tuple(n_points);
    bn::ndarray result = bn::zeros(shape, dt);
    uint8_t* changes = reinterpret_cast<uint8_t *>(result.get_data());

    // Detect changes.
    int row_stride = scan.strides(0) / sizeof(double);
    int col_stride = scan.strides(1) / sizeof(double);
    double *row_iter = reinterpret_cast<double *>(scan.get_data());
    for (int pt_idx = 0; pt_idx < n_points; ++pt_idx, row_iter += row_stride) {
        bool change = true;
        double *col_iter = row_iter;
        point_t p(*col_iter, 
                  *(col_iter + col_stride), 
                  *(col_iter + 2 * col_stride));

        grid_t::const_iterator it = grid_.find( grid_t::point_type( p.bearing(), p.elevation() ) );
        if( it == grid_.end() ) { 
            change = false; 
        } else {
            const cell::entry* q = it->second.trace( p, angle_threshold_, range_threshold_, invert );
            if( !q ) { change = false; }
        }
        changes[pt_idx] = change;
    }

    return result;
}

bn::ndarray detect_change(const bn::ndarray& ref, const bn::ndarray& scan,
                          double angle_threshold, double range_threshold) 
{
    ChangeDetector cd(ref, angle_threshold, range_threshold);
    return cd.GetChanges(scan);
}
