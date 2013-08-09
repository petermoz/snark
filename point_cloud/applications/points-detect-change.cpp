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


/// @author james underwood
/// @author vsevolod vlaskine

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <cmath>
#include <string.h>
#include <fstream>
#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include <snark/point_cloud/detect_change.h>

//#include <google/profiler.h>

snark::voxel_map< int, 2 >::point_type resolution;
static bool verbose;

void usage( bool long_help = false )
{
    std::cerr << std::endl;
    std::cerr << "a quick-n-dirty application; the interface is way too rigid" << std::endl;
    std::cerr << "load a point cloud in polar from file; for each point on stdin output whether it is blocked in the ray or not by points of the point cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-detect-change reference_points.csv [<options>] > points.marked-as-blocked.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --long-help: more help" << std::endl;
    std::cerr << "    --range-threshold,-r=<value>: if present, output only the points" << std::endl;
    std::cerr << "                                  that have reference points nearer than range + range-threshold" << std::endl;
    std::cerr << "    --angle-threshold,-a=<value>: angular radius in radians" << std::endl;
    std::cerr << "    --verbose,-v: more debug output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields: r,b,e: range, bearing, elevation; default: r,b,e" << std::endl;
    if( long_help )
    {
        std::cerr << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    std::cerr << std::endl;
    exit( 1 );
}


int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv );
        if( options.exists( "--help,-h" ) ) { usage(); }
        if( options.exists( "--long-help" ) ) { usage( true ); }
        verbose = options.exists( "--verbose,-v" );
        comma::csv::options csv( options, "range,bearing,elevation" );
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i] == "r" ) { v[i] = "range"; }
            else if( v[i] == "b" ) { v[i] = "bearing"; }
            else if( v[i] == "e" ) { v[i] = "elevation"; }
        }
        csv.fields = comma::join( v, ',' );
        csv.full_xpath = false;
        double threshold = options.value< double >( "--angle-threshold,-a" );
        boost::optional< double > range_threshold = options.optional< double >( "--range-threshold,-r" );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v", "--binary,-b,--delimiter,-d,--fields,-f,--range-threshold,-r,--angle-threshold,-a" );
        if( unnamed.empty() ) { std::cerr << "points-detect-change: please specify file with the reference point cloud" << std::endl; return 1; }
        if( unnamed.size() > 1 ) { std::cerr << "points-detect-change: expected file with the reference point cloud, got: " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        #ifdef WIN32
            std::ios::openmode mode = 0;
            if( csv.binary() )
            {
                mode |= std::ios::binary;
                _setmode( _fileno( stdin ), _O_BINARY );
                _setmode( _fileno( stdout ), _O_BINARY );
            }
            std::ifstream ifs( unnamed[0].c_str(), mode );
        #else
            std::ifstream ifs( unnamed[0].c_str() );
        #endif
        if( !ifs.is_open() ) { std::cerr << "points-detect-change: failed to open \"" << unnamed[0] << "\"" << std::endl; return 1; }
        comma::csv::input_stream< point_t > ifstream( ifs, csv );
        typedef snark::voxel_map< cell, 2 > grid_t;
        resolution = grid_t::point_type( threshold, threshold );
        grid_t grid( resolution );
        if( verbose ) { std::cerr << "points-detect-change: loading reference point cloud..." << std::endl; }
        comma::signal_flag is_shutdown;
        comma::uint64 index = 0;
        //{ ProfilerStart( "points-detect-change.prof" );
        std::deque< std::vector< char > > buffers;
        while( ifs.good() && !ifs.eof() && !is_shutdown )
        {
            const point_t* p = ifstream.read();
            if( !p ) { break; }
            cell::entry entry( *p, index );
            for( int i = -1; i < 2; ++i )
            {
                for( int j = -1; j < 2; ++j )
                {
                    double bearing = p->bearing() + threshold * i;
                    if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                    else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                    double elevation = p->elevation() + threshold * j;
                    grid_t::iterator it = grid.touch_at( grid_t::point_type( bearing, elevation ) );
                    it->second.add( entry ); //it->second.add_to_grid( entry );
                }
            }
            buffers.push_back( std::vector< char >() ); // todo: quick and dirty; use memory map instead?
            if( csv.binary() )
            {
                static unsigned int s = ifstream.binary().binary().format().size();
                buffers.back().resize( s );
                ::memcpy( &buffers.back()[0], ifstream.binary().last(), s );
            }
            else
            {
                std::string s = comma::join( ifstream.ascii().last(), csv.delimiter );
                buffers.back().resize( s.size() );
                ::memcpy( &buffers.back()[0], &s[0], s.size() );
            }
            ++index;
        }
        if( verbose ) { std::cerr << "points-detect-change: loaded reference point cloud: " << index << " points in a grid of size " << grid.size() << " voxels" << std::endl; }
        comma::csv::input_stream< point_t > istream( std::cin, csv );
        while( std::cin.good() && !std::cin.eof() && !is_shutdown )
        {
            const point_t* p = istream.read();
            if( !p ) { break; }
            grid_t::const_iterator it = grid.find( grid_t::point_type( p->bearing(), p->elevation() ) );
            if( it == grid.end() ) { continue; }
            const cell::entry* q = it->second.trace( *p, threshold, range_threshold );
            if( !q ) { continue; }
            if( csv.binary() )
            {
                static unsigned int is = istream.binary().binary().format().size();
                std::cout.write( istream.binary().last(), is );
                static unsigned int fs = ifstream.binary().binary().format().size();
                std::cout.write( &buffers[q->index][0], fs );
            }
            else
            {
                std::cout << comma::join( istream.ascii().last(), csv.delimiter )
                          << csv.delimiter
                          << std::string( &buffers[q->index][0], buffers[q->index].size() ) << std::endl;
            }
        }
        //} ProfilerStop();
        if( is_shutdown ) { std::cerr << "points-detect-change: caught signal" << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "points-detect-change: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "points-detect-change: unknown exception" << std::endl;
    }
    return 1;
}
