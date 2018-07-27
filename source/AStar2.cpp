#include "AStar2.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>

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
    coordinates = coordinates_;
    G = H = 0;
}


Generator::Generator()
{
    _allow_5x5_search = true;
    setHeuristic(&Heuristic::manhattan);
    _directions = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 },

        { -2, -2 }, { -2, -1 }, { -2, 0 }, { -2, 1 }, { -2, 2 },
        { -1, -2 },                                   { -1, 2 },
        { -0, -2 },                                   {  0, 2 },
        {  1, -2 },                                   {  1, 2 },
        {  2, -2 }, {  2, -1 }, {  2, 0 }, {  2, 1 }, {  2, 2 }
    };

    _direction_cost = {
        10, 10, 10, 10,
        14, 14, 14, 14,

        28, 24, 20, 24, 28,
        24,             24,
        20,             20,
        24,             20,
        28, 24, 20, 24, 28
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
        _world_grid[i] = data[i] < 100 ? OBSTACLE : EMPTY;
    }

    _open_set_2Dmap.resize(width*height, -1);
}

void Generator::setHeuristic(HeuristicFunction heuristic_)
{
    _heuristic = std::bind(heuristic_, _1, _2);
}


void Generator::clean()
{
    for(auto& val: _world_grid)
    {
        val = ( val == CLOSED ) ? EMPTY : val;
    }
    _memory_storage.clear();
    _open_set.clear();
    _closed_set.clear();
    _open_set_2Dmap.clear();
    _open_set_2Dmap.resize( _world_width*_world_height, -1 );
}

NodePtr Generator::findMinScoreInOpenSet()
{
    if( _open_set.empty() ) return -1;

    NodePtr current = ( _open_set.begin()->second );
    _open_set.erase( _open_set.begin() );
    return current;
}

CoordinateList Generator::findPath(Vec2i source_, Vec2i target_)
{
    clean();

    auto toIndex = [this](Vec2i pos) -> size_t
    { return static_cast<size_t>(_world_width*pos.y + pos.x); };

    NodePtr current_ptr = -1;
    _open_set.insert( {0, newNode(source_,-1) } );
    _open_set_2Dmap[ toIndex(source_) ] = _open_set.begin()->second;

    bool solution_found = false;

    while (! _open_set.empty() )
    {
        current_ptr = findMinScoreInOpenSet();
        Vec2i coordinates = getNode(current_ptr)->coordinates;

        _open_set_2Dmap[ toIndex( coordinates ) ] = -1;
        _closed_set.push_back( current_ptr );

        if (coordinates == target_) {
            solution_found = true;
            break;
        }

        grid( coordinates ) = CLOSED;

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
            end_i = 8 + 16;
            for (int i=0; i<8 && can_do_jump_16; i++)
            {
               // grid( coordinates + _directions[i] ) = CLOSED;
            }
        }

        for (uint i = start_i; i < end_i; ++i)
        {
            Node* curr_node = getNode(current_ptr);
            Vec2i newCoordinates(coordinates + _directions[i]);
            if (detectCollision(newCoordinates) ||
                grid( newCoordinates ) == CLOSED ) {
                continue;
            }

            uint totalCost = curr_node->G + _direction_cost[i];

            size_t newIndex = toIndex(newCoordinates);
            NodePtr successor_ptr = _open_set_2Dmap[ newIndex ];

            if (successor_ptr == -1) {
                successor_ptr = newNode(newCoordinates, current_ptr );
                auto successor = getNode(successor_ptr);
                successor->G = totalCost;
                successor->H = _heuristic(successor->coordinates, target_);
                _open_set.insert( { successor->getScore(), successor_ptr } );
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
    if( solution_found ){
        while (current_ptr != -1) {
            path.push_back( getNode(current_ptr)->coordinates);
            current_ptr = getNode(current_ptr)->parent;
        }
    }

    if( !solution_found )
    {
        std::cout << "found " << solution_found <<
                     " closed set " << _closed_set.size() <<
                     " open set " << _open_set.size() << std::endl;
    }

    return path;
}

NodePtr Generator::findNodeOnList(NodeSet& nodes, Vec2i coordinates)
{
    for (auto node : nodes) {
        if ( getNode(node)->coordinates == coordinates) {
            return node;
        }
    }
    return -1;
}

NodePtr Generator::newNode(Vec2i coord, NodePtr parent)
{   
    _memory_storage.emplace_back( Node(coord, parent) );
    return static_cast<int>(_memory_storage.size())-1 ;
}

bool Generator::detectCollision(Vec2i coordinates)
{
    if (coordinates.x < 0 || coordinates.x >= _world_width ||
            coordinates.y < 0 || coordinates.y >= _world_height ||
            grid(coordinates) == OBSTACLE ){
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
