#include <snark/point_cloud/detect_change.h>


double abs_bearing_distance_( double m, double b ) // quick and dirty
{
    double m1 = m < 0 ? m + ( M_PI * 2 ) : m;
    double b1 = b < 0 ? b + ( M_PI * 2 ) : b;
    return std::abs( m1 - b1 );
}

bool bearing_between_( double b, double min, double max )
{
    return comma::math::equal( abs_bearing_distance_( b, min ) + abs_bearing_distance_( b, max ), abs_bearing_distance_( min, max ) ); // quick and dirty: watch performance
}

double bearing_min_( double m, double b )
{
    double m1 = m < 0 ? m + ( M_PI * 2 ) : m;
    double b1 = b < 0 ? b + ( M_PI * 2 ) : b;
    return comma::math::less( m1, b1 ) ? m : b;
}

double bearing_max_( double m, double b )
{
    double m1 = m < 0 ? m + ( M_PI * 2 ) : m;
    double b1 = b < 0 ? b + ( M_PI * 2 ) : b;
    return comma::math::less( b1, m1 ) ? m : b;
}



void cell::add( const entry& p )
{
    if( points.size() == points.capacity() ) { points.reserve( 64 ); } // quick and dirty
    points.push_back( p );
}

const cell::entry* cell::trace( const point_t& p, double threshold, boost::optional< double > range_threshold ) const
{
    const entry* e = NULL;
    static const double threshold_square = threshold * threshold; // static: quick and dirty
    boost::optional< point_t > min;
    boost::optional< point_t > max;
    for( std::size_t i = 0; i < points.size(); ++i )
    {
        double db = abs_bearing_distance_( p.bearing(), points[i].point.bearing() );
        double de = p.elevation() - points[i].point.elevation();
        if( ( db * db + de * de ) > threshold_square ) { continue; }
        if( range_threshold && points[i].point.range() < ( p.range() + *range_threshold ) ) { return NULL; }
        if( min ) // todo: quick and dirty, fix point_tRBE and use extents
        {
            if( points[i].point.range() < min->range() )
            {
                min->range( points[i].point.range() );
                e = &points[i];
            }
            min->bearing( bearing_min_( min->bearing(), points[i].point.bearing() ) );
            min->elevation( std::min( min->elevation(), points[i].point.elevation() ) );
            max->bearing( bearing_max_( max->bearing(), points[i].point.bearing() ) );
            max->elevation( std::max( max->elevation(), points[i].point.elevation() ) );
        }
        else
        {
            e = &points[i];
            min = max = points[i].point;
        }
    }
    return    !min
           || !bearing_between_( p.bearing(), min->bearing(), max->bearing() )
           || !comma::math::less( min->elevation(), p.elevation() )
           || !comma::math::less( p.elevation(), max->elevation() ) ? NULL : e;
}


