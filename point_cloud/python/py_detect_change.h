#include <boost/python.hpp>
#include <boost/numpy.hpp>
#include <snark/point_cloud/detect_change.h>

namespace bp = boost::python;
namespace bn = boost::numpy;

typedef snark::voxel_map<cell, 2 > grid_t;

class ChangeDetector
{
public:
    ChangeDetector(const bn::ndarray& ref, double angle_threshold, 
                   double range_threshold);

    bn::ndarray GetChanges(const bn::ndarray& scan);

private:
    grid_t grid_;
    double angle_threshold_;
    double range_threshold_;
};

bn::ndarray detect_change(const bn::ndarray& ref, const bn::ndarray& scan,
                          double angle_threshold, double range_threshold);
