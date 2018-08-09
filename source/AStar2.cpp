#include "AStar2.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <cinttypes>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>

using namespace std::placeholders;

namespace AStar2{

bool Coord2D::operator == (const Coord2D& coordinates)
{
    return (x == coordinates.x && y == coordinates.y);
}

Coord2D operator + (const Coord2D& left_, const Coord2D& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

Node::Node(Coord2D coordinates_)
{
    coord_x = coordinates_.x;
    coord_y = coordinates_.y;
    G = 0;
}


Generator::Generator():
    _open_set( CompareScore() )
{
    _allow_5x5_search = false;
    setHeuristic(&Heuristic::manhattan);
    _directions = {
        { -1, -1 },  { 0, -1 }, { 1, -1 },
        { -1,  0 },             { 1,  0 },
        { -1,  1 },  { 0, 1 },  { 1,  1 },

//        { -2, -2 }, { -1, -2 }, { 0, -2 }, { 1, -2 }, { 2, -2 },
//        { -2, -1 },                                   { 2, -1 },
//        { -2,  0 },                                   { 2,  0 },
//        { -2,  1 },                                   { 2,  1 },
//        { -2,  2 }, { -1,  2 }, { 0,  2 }, { 1,  2 }, { 2,  2 }

        { -2, -2 }, { -2, -1 }, { -2, 0 }, { -2, 1 }, { -2, 2 },
        { -1, -2 },                                   { -1, 2 },
        { -0, -2 },                                   {  0, 2 },
        {  1, -2 },                                   {  1, 2 },
        {  2, -2 }, { 2,  -1 }, { 2,  0 }, { 2,  1 }, {  2, 2 }
    };

    _direction_cost = {
        14, 10, 14,
        10,     10,
        14, 10, 14,

        28, 22, 20, 22, 28,
        22,             22,
        20,             20,
        22,             22,
        28, 22, 20, 22, 28
    };
}

Generator::~Generator()
{
}


void Generator::setWorldData(int width, int height, const uint8_t *data)
{
    _world_width  = width;
    _world_height = height;
    _world_grid.resize(width*height);
    for (size_t i=0; i<width*height; i++ )
    {
        if(  data[i] < 20 )
        {
            _world_grid[i] = OBSTACLE;
        }
        if(  data[i] > 235 )
        {
            _world_grid[i] = EMPTY;
        }
        else{
            _world_grid[i] = data[i];
        }
    }
}

void Generator::setHeuristic(HeuristicFunction heuristic_)
{
    _heuristic = std::bind(heuristic_, _1, _2);
}


void Generator::clean()
{
    while( !_open_set.empty() )
    {
        _open_set.pop();
    }
    _cost_map.clear();
    _cost_map.resize( _world_width*_world_height,
                      std::numeric_limits<float>::infinity() );

    _path_map.resize(_world_width*_world_height);

    _closed_grid.clear();
    _closed_grid.resize(_world_width*_world_height, false);
}


CoordinateList Generator::findPath(Coord2D startPos, Coord2D goalPos)
{
    clean();

    auto toIndex = [this](Coord2D pos) -> size_t
    { return static_cast<size_t>(_world_width*pos.y + pos.x); };

    const int startIndex = toIndex(startPos);
    const int goalIndex = toIndex(goalPos);

    _open_set.push( {0, Node(startPos) } );
    _cost_map[ startIndex ] = 0.0;

    bool solution_found = false;

    while (! _open_set.empty() )
    {
        Node current = _open_set.top().second;
        _open_set.pop();

        Coord2D coordinates = current.coordinates();

        if (coordinates == goalPos) {
            solution_found = true;
            break;
        }
        _closed_grid[ toIndex(coordinates) ] = true;

        bool can_do_jump_16 = _allow_5x5_search;
        for (int i=0; i<8 && can_do_jump_16; i++)
        {
            can_do_jump_16 = ! detectCollision( coordinates + _directions[i] );
        }

        uint start_i = 0;
        uint end_i   = 8;
        if( can_do_jump_16 )
        {
            start_i = 8;
            end_i   = 8 + 16;
        }

        for (uint i = start_i; i < end_i; ++i)
        {
            Coord2D newCoordinates(coordinates + _directions[i]);
            size_t newIndex = toIndex(newCoordinates);

            if (detectCollision(newCoordinates) ||
                _closed_grid[newIndex] ) {
                continue;
            }

            float pixel_color =  worldGrid( newCoordinates );
            float factor = 1.0f + static_cast<float>(EMPTY - pixel_color) / 50.0f;
            uint new_cost = current.G + _direction_cost[i] * factor;

            if( new_cost < _cost_map[ newIndex ])
            {
                Node  successor = Node(newCoordinates );
                successor.G = new_cost;
                auto H = _heuristic(successor.coordinates(), goalPos);

                _open_set.push( { successor.G + H, successor } );
                _cost_map[ newIndex ] = new_cost;
                _path_map[ newIndex ] = toIndex( current.coordinates() );
            }
        }
    }

    CoordinateList path;
    if( solution_found )
    {
        int index = goalIndex;
        while (index != startIndex)
        {
            path.push_back( { index % _world_width, index / _world_width} );
            index = _path_map[index];
        }
    }

    if( !solution_found )
    {
        std::cout << "Solution not found\n" <<
                     " open set size= " << _open_set.size() << std::endl;
    }

    return path;
}

void Generator::exportPPM(const char *filename, CoordinateList* path)
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);

    char header[100];
    sprintf(header, "P6\n# Done by Davide\n%d %d\n255\n", _world_width, _world_height );
    outfile.write(header, strlen(header));

    std::vector<uint8_t> image( _world_width * _world_height * 3);

    int line_size = _world_width * 3;

    auto toIndex = [line_size](int x, int y) { return y*line_size + (x*3); };

    for (int y=0; y<_world_height; y++)
    {
        for (int x=0; x<_world_width; x++)
        {
            if( worldGrid({x,y}) == OBSTACLE )
            {
                uint8_t color[] = {0,0,0};
                mempcpy( &image[ toIndex(x,y) ], color, 3 );
            }
            else if( _closed_grid[ y*_world_width + x ] )
            {
                uint8_t color[] = {255,222,222};
                mempcpy( &image[ toIndex(x,y) ], color, 3 );
            }
            else{
                uint8_t color[] = {255,255,255};
                mempcpy( &image[ toIndex(x,y) ], color, 3 );
            }
        }
    }

    if( path )
    {
        for (const auto& point: *path)
        {
            uint8_t color[] = {50,50,250};
            mempcpy( &image[ toIndex(point.x, point.y) ], color, 3 );
        }
    }

    outfile.write( reinterpret_cast<char*>(image.data()), image.size() );
    outfile.close();
}


bool Generator::detectCollision(Coord2D coordinates)
{
    if (coordinates.x < 0 || coordinates.x >= _world_width ||
            coordinates.y < 0 || coordinates.y >= _world_height ||
            worldGrid(coordinates) == OBSTACLE ){
        return true;
    }
    return false;
}

Coord2D Heuristic::getDelta(Coord2D source, Coord2D target)
{
    return{ abs(source.x - target.x),  abs(source.y - target.y) };
}

uint Heuristic::manhattan(Coord2D source, Coord2D target)
{
    auto delta = getDelta(source, target);
    return static_cast<uint>(10 * (delta.x + delta.y));
}

uint Heuristic::euclidean(Coord2D source, Coord2D target)
{
    auto delta = getDelta(source, target);
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

uint Heuristic::octagonal(Coord2D source, Coord2D target)
{
    auto delta = getDelta(source, target);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

}
