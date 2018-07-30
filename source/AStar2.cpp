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

bool Vec2i::operator == (const Vec2i& coordinates)
{
    return (x == coordinates.x && y == coordinates.y);
}

Vec2i operator + (const Vec2i& left_, const Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

Node::Node(Vec2i coordinates_, NodePtr parent_)
{
    parent = parent_;
    coord_x = coordinates_.x;
    coord_y = coordinates_.y;
    G = H = 0;
}


Generator::Generator():
    _open_set( CompareScore() )
{
    _allow_5x5_search = true;
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

    _memory_storage.reserve(1000);
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
    _memory_storage.clear();
    while( !_open_set.empty() )
    {
        _open_set.pop();
    }
    _open_set_2Dmap.clear();
    _open_set_2Dmap.resize( _world_width*_world_height, -1 );

    _closed_grid.clear();
    _closed_grid.resize( _world_width*_world_height, false );
}

NodePtr Generator::findMinScoreInOpenSet()
{
    if( _open_set.empty() ) return -1;

    NodePtr current = _open_set.top().second;
    _open_set.pop();
    return current;
}

CoordinateList Generator::findPath(Vec2i source_, Vec2i target_)
{
    clean();

    auto toIndex = [this](Vec2i pos) -> size_t
    { return static_cast<size_t>(_world_width*pos.y + pos.x); };

    NodePtr current_ptr = -1;
    _open_set.push( {0, newNode(source_,-1) } );
    _open_set_2Dmap[ toIndex(source_) ] = _open_set.top().second;

    bool solution_found = false;

    while (! _open_set.empty() )
    {
        current_ptr = findMinScoreInOpenSet();
        Vec2i coordinates = getNode(current_ptr)->coordinates();

        _open_set_2Dmap[ toIndex( coordinates ) ] = -1;

        if (coordinates == target_) {
            solution_found = true;
            break;
        }

        setClosedGrid( coordinates, true );

        bool can_do_jump_16 = _allow_5x5_search;
        for (int i=0; i<8 && can_do_jump_16; i++)
        {
            can_do_jump_16 = ! detectCollision( coordinates + _directions[i] );
        }

        uint end_i   = 8;
        if( can_do_jump_16 )
        {
            end_i = 8 + 16;
        }

        for (uint i = 0; i < end_i; ++i)
        {
            Node* curr_node = getNode(current_ptr);
            Vec2i newCoordinates(coordinates + _directions[i]);

            if (detectCollision(newCoordinates) ||
                closedGrid( newCoordinates ) ) {
                continue;
            }
            double pixel_color =  worldGrid( newCoordinates );
            double factor = 1.0 + static_cast<double>((EMPTY - pixel_color) / 20) / 2.0;
            uint totalCost = curr_node->G + _direction_cost[i] * factor;

            size_t newIndex = toIndex(newCoordinates);
            NodePtr successor_ptr = _open_set_2Dmap[ newIndex ];

            if (successor_ptr == -1) {
                successor_ptr = newNode(newCoordinates, current_ptr );
                auto successor = getNode(successor_ptr);
                successor->G = totalCost;
                successor->H = _heuristic(successor->coordinates(), target_);
                _open_set.push( { successor->getScore(), successor_ptr } );
                _open_set_2Dmap[ newIndex ] = successor_ptr;
            }
            else if (totalCost < getNode(successor_ptr)->G) {
                auto successor = getNode(successor_ptr);
                successor->parent = current_ptr;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
   // if( solution_found )
    {
        while (current_ptr != -1) {
            path.push_back( getNode(current_ptr)->coordinates());
            current_ptr = getNode(current_ptr)->parent;
        }
    }

    if( !solution_found )
    {
        std::cout << "found " << solution_found <<
                     " open set " << _open_set.size() << std::endl;
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
            else if( closedGrid( {x,y}) )
            {
                uint8_t color[] = {255,180,180};
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

NodePtr Generator::findNodeOnList(NodeSet& nodes, Vec2i coordinates)
{
    for (auto node : nodes) {
        if ( getNode(node)->coordinates() == coordinates) {
            return node;
        }
    }
    return -1;
}

NodePtr Generator::newNode(Vec2i coord, NodePtr parent)
{   
    _memory_storage.emplace_back( Node(coord, parent) );
    return static_cast<NodePtr>(_memory_storage.size())-1 ;
}

bool Generator::detectCollision(Vec2i coordinates)
{
    if (coordinates.x < 0 || coordinates.x >= _world_width ||
            coordinates.y < 0 || coordinates.y >= _world_height ||
            worldGrid(coordinates) == OBSTACLE ){
        return true;
    }
    return false;
}

Vec2i Heuristic::getDelta(Vec2i source, Vec2i target)
{
    return{ abs(source.x - target.x),  abs(source.y - target.y) };
}

uint Heuristic::manhattan(Vec2i source, Vec2i target)
{
    auto delta = getDelta(source, target);
    return static_cast<uint>(10 * (delta.x + delta.y));
}

uint Heuristic::euclidean(Vec2i source, Vec2i target)
{
    auto delta = getDelta(source, target);
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

uint Heuristic::octagonal(Vec2i source, Vec2i target)
{
    auto delta = getDelta(source, target);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

}
