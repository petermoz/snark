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
bn::ndarray partition(bn::ndarray scan, 
                      bp::list resolution,
                      double min_density,
                      int min_id,
                      int min_points_per_voxel,
                      int min_points_per_partition,
                      int min_voxels_per_partition) {

    // Check that scan input is as expected.
    if(scan.get_dtype() != bn::dtype::get_builtin<double>()) {
        PyErr_SetString(PyExc_TypeError, "Incorrect array data type (expected double)");
        bp::throw_error_already_set();
    }
    if(scan.get_nd() != 2 || scan.shape(1) != 3) {
        PyErr_SetString(PyExc_ValueError, "Incorrect dimensions (expected N x 3)");
        bp::throw_error_already_set();
    }

    const int n_points = scan.shape(0);
    boost::optional< snark::math::closed_interval< double, 3 > > extents;

    // Load the points and compute extents.
    std::vector<InputPoint> points;
    points.reserve(n_points);

    {
        const int row_stride = scan.strides(0) / sizeof(double);
        const int col_stride = scan.strides(1) / sizeof(double);

        // Compute extents of input.
        double *row_iter = reinterpret_cast<double *>(scan.get_data());
        for (int pt_idx = 0; pt_idx < n_points; ++pt_idx, row_iter += row_stride) {
            double *col_iter = row_iter;
            InputPoint pt;
            pt.point << *col_iter, 
                        *(col_iter + col_stride), 
                        *(col_iter + 2 * col_stride);
            if( !extents ) {
                extents = snark::math::closed_interval< double, 3 >( pt.point );
            } else {
                extents = extents->hull(pt.point);
            }
            points.push_back(pt);
        }
    }

    // Insert all points into the paritioning grid.
    Eigen::Vector3d resolution3;
    resolution3 << bp::extract<double>(resolution[0]),
                   bp::extract<double>(resolution[1]),
                   bp::extract<double>(resolution[2]);
    snark::partition partition(*extents, resolution3, min_points_per_voxel);
    for(size_t i = 0; i < points.size(); ++i)
    {
        points[i].partition = &partition.insert(points[i].point);
    }

    // Create result array.
    bn::dtype dt = bn::dtype::get_builtin<uint32_t>();
    bp::tuple shape = bp::make_tuple(n_points);
    bn::ndarray result = bn::zeros(shape, dt);

    {
        uint32_t* result_iter = reinterpret_cast<uint32_t*>(result.get_data());
        const int row_stride = result.strides(0) / sizeof(uint32_t);

        // Compute the output ids.
        partition.commit(min_voxels_per_partition, min_points_per_partition, min_id);
        for(int pt_idx = 0; pt_idx < n_points; ++pt_idx, result_iter += row_stride) 
        {
            const InputPoint& point = points[pt_idx];
            comma::uint32 id = ID_INVALID; // Not partitioned.
            if( point.partition && *point.partition ) {
                id = **point.partition;
            }
            *result_iter = id;
        }
    }

    return result;
}
