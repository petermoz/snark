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

#include <boost/python.hpp>
#include <boost/numpy.hpp>
#include <snark/point_cloud/python/py_detect_change.h>
#include <snark/point_cloud/python/py_partition.h>

namespace bp = boost::python;
namespace bn = boost::numpy;

BOOST_PYTHON_MODULE(snark_py_point_cloud)
{
    bn::initialize();

    // Change detection.
    bp::class_<ChangeDetector,
       boost::shared_ptr<ChangeDetector>, boost::noncopyable>("ChangeDetector",
            bp::init<bn::ndarray, double, double>(
                (bp::arg("reference"), bp::arg("angle_threshold"), bp::arg("range_threshold"))
            ))
        .def("get_changes", &ChangeDetector::GetChanges, (bp::arg("scan"), bp::arg("invert")=false))
    ;

    bp::def("detect_change", detect_change,
            (bp::arg("reference"), bp::arg("scan"), bp::arg("angle_threshold"), bp::arg("range_threshold")));

    
    // Partitioning.
    bp::def("partition", partition,
            (bp::arg("scan"), 
             bp::arg("resolution"),
             bp::arg("min_density") = 0.0,
             bp::arg("min_id") = 0,
             bp::arg("min_points_per_voxel") = 1,
             bp::arg("min_points_per_partition") = 1,
             bp::arg("min_voxels_per_partition") = 1));
    bp::scope().attr("ID_INVALID") = ID_INVALID;
}

