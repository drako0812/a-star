#include "AStar2.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <cinttypes>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

using namespace std::placeholders;

namespace AStar{

bool Coord2D::operator == (const Coord2D& coordinates)
{
    return (x == coordinates.x && y == coordinates.y);
}

Coord2D operator + (const Coord2D& left_, const Coord2D& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}


PathFinder::PathFinder():
    _open_set( CompareScore() )
{
    _allow_5x5_search = false;
    setHeuristic(&Heuristic::manhattan);
    _directions = {{
        { -1, -1 },  { 0, -1 }, { 1, -1 },
        { -1,  0 },             { 1,  0 },
        { -1,  1 },  { 0, 1 },  { 1,  1 },

        { -2, -2 }, { -2, -1 }, { -2, 0 }, { -2, 1 }, { -2, 2 },
        { -1, -2 },                                   { -1, 2 },
        { -0, -2 },                                   {  0, 2 },
        {  1, -2 },                                   {  1, 2 },
        {  2, -2 }, { 2,  -1 }, { 2,  0 }, { 2,  1 }, {  2, 2 }
    }};

    _direction_cost = {{
        14, 10, 14,
        10,     10,
        14, 10, 14,

        28, 22, 20, 22, 28,
        22,             22,
        20,             20,
        22,             22,
        28, 22, 20, 22, 28
    }};
}

PathFinder::~PathFinder()
{
}


void PathFinder::setWorldData(int width, int height, const uint8_t *data)
{
    _world_width  = width;
    _world_height = height;
    _gridmap.resize(width*height);

    for (size_t i=0; i<width*height; i++ )
    {
        if(  data[i] < 20 )
        {
            _gridmap[i].world = OBSTACLE;
        }
        if(  data[i] > 235 )
        {
            _gridmap[i].world = EMPTY;
        }
        else{
            _gridmap[i].world = data[i];
        }
    }
}

void PathFinder::setHeuristic(HeuristicFunction heuristic_)
{
    _heuristic = std::bind(heuristic_, _1, _2);
}


void PathFinder::clean()
{
    while( !_open_set.empty() )
    {
        _open_set.pop();
    }

    for(Cell& cell: _gridmap)
    {
        cell.cost_G = std::numeric_limits<float>::infinity();
        cell.path_parent_index = 0.0;
        cell.already_visited = false;
    }
}


CoordinateList PathFinder::findPath(Coord2D startPos, Coord2D goalPos)
{
    clean();

    auto toIndex = [this](Coord2D pos) -> int
    { return static_cast<int>(_world_width*pos.y + pos.x); };

    const int startIndex = toIndex(startPos);
    const int goalIndex = toIndex(goalPos);

    _open_set.push( {0, startPos } );
    _gridmap[startIndex].cost_G = 0.0;

    bool solution_found = false;

    while (! _open_set.empty() )
    {
        Coord2D currentCoord = _open_set.top().second;
        _open_set.pop();

        if (currentCoord == goalPos) {
            solution_found = true;
            break;
        }
        int currentIndex = toIndex(currentCoord);
        Cell& currentCell = _gridmap[ currentIndex ];
        currentCell.already_visited = true;

        bool can_do_jump_16 = _allow_5x5_search;
        for (int i=0; i<8 && can_do_jump_16; i++)
        {
            can_do_jump_16 = ! detectCollision( currentCoord + _directions[i] );
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
            Coord2D newCoordinates(currentCoord + _directions[i]);

            if (detectCollision(newCoordinates)) {
                continue;
            }
            size_t newIndex = toIndex(newCoordinates);
            Cell& newCell = _gridmap[newIndex];

            if ( newCell.already_visited ) {
                continue;
            }

            float pixel_color =  newCell.world;
            float factor = 1.0f + static_cast<float>(EMPTY - pixel_color) / 50.0f;
            float new_cost = currentCell.cost_G + _direction_cost[i] * factor;

            if( new_cost < newCell.cost_G)
            {
                float H = _heuristic( newCoordinates, goalPos );
                _open_set.push( { new_cost + H, newCoordinates } );
                newCell.cost_G = new_cost;
                newCell.path_parent_index = currentIndex;
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
            index = _gridmap[index].path_parent_index;
        }
    }
    else
    {
        std::cout << "Solution not found\n" <<
                     " open set size= " << _open_set.size() << std::endl;
    }

    return path;
}

void PathFinder::exportPPM(const char *filename, CoordinateList* path)
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
            if( cell({x,y}).world == OBSTACLE )
            {
                uint8_t color[] = {0,0,0};
                mempcpy( &image[ toIndex(x,y) ], color, 3 );
            }
            else if( _gridmap[ y*_world_width + x ].already_visited )
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


bool PathFinder::detectCollision(Coord2D coordinates)
{
    if (coordinates.x < 0 || coordinates.x >= _world_width ||
            coordinates.y < 0 || coordinates.y >= _world_height ||
            cell(coordinates).world == OBSTACLE ){
        return true;
    }
    return false;
}

Coord2D Heuristic::getDelta(Coord2D source, Coord2D target)
{
    return{ abs(source.x - target.x),  abs(source.y - target.y) };
}

float Heuristic::manhattan(Coord2D source, Coord2D target)
{
    auto delta = getDelta(source, target);
    return static_cast<uint>(10 * (delta.x + delta.y));
}

float Heuristic::euclidean(Coord2D source, Coord2D target)
{
    auto delta = getDelta(source, target);
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

float Heuristic::octagonal(Coord2D source, Coord2D target)
{
    auto delta = getDelta(source, target);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

}
