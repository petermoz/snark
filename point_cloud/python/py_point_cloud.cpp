#include <boost/python.hpp>
#include <boost/numpy.hpp>
#include <snark/point_cloud/python/py_detect_change.h>

namespace bp = boost::python;
namespace bn = boost::numpy;

BOOST_PYTHON_MODULE(snark_py_point_cloud)
{
    bn::initialize();

    bp::class_<ChangeDetector>("ChangeDetector",
            bp::init<bn::ndarray, double, double>())
        .def("get_changes", &ChangeDetector::GetChanges)
    ;

    bp::def("detect_change", detect_change);
}

