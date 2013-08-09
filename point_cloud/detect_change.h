#include <snark/math/range_bearing_elevation.h>
#include <snark/point_cloud/voxel_map.h>
#include <snark/visiting/traits.h>


typedef snark::range_bearing_elevation point_t;

double abs_bearing_distance_( double m, double b );
bool bearing_between_( double b, double min, double max );
double bearing_min_( double m, double b );
double bearing_max_( double m, double b );

struct cell {
    struct entry
    {
        point_t point;
        comma::uint64 index;
        entry() {}
        entry( const point_t& point, comma::uint64 index ) : point( point ), index( index ) {}
    };
    
    std::vector< entry > points;

    void add( const entry& p );
    const entry* trace( const point_t& p, double threshold, boost::optional< double > range_threshold ) const;
};
