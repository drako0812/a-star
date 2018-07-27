/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR2_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR2_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>

namespace AStar2
{
    struct Vec2i
    {
        int x, y;
        bool operator == (const Vec2i& coordinates);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    typedef int32_t NodePtr;
    typedef std::pair<uint,NodePtr> ScoreNodePair;

    struct CompareScore
    {
      bool operator() (const ScoreNodePair& a,
                       const ScoreNodePair& b)
        {
          return a.first > b.first;
        }
    };

    struct Node
    {
        uint32_t G, H;
        uint16_t coord_x, coord_y;
        NodePtr parent;

        Node(Vec2i coord = {0,0}, NodePtr parent = -1);

        Vec2i coordinates() const
        {
            return {coord_x, coord_y};
        }

        uint getScore() const
        {
            return G + H;
        }

        bool operator <(const Node& other) const
        {
            return getScore() < other.getScore();
        }
    };

    using NodeSet = std::vector<NodePtr>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates);
        NodePtr  findNodeOnList(NodeSet &nodes, Vec2i coordinates);
        NodePtr  newNode(Vec2i coord, NodePtr parent);

    public:
        Generator();
        ~Generator();

        void setWorldData(int width, int height, const uint8_t *data);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList  findPath(Vec2i source_, Vec2i target_);

        const uint8_t& worldGrid(Vec2i coordinates_) const
        {
            return _world_grid[coordinates_.y*_world_width + coordinates_.x];
        }

        uint8_t& worldGrid(Vec2i coordinates_)
        {
            return _world_grid[coordinates_.y*_world_width + coordinates_.x];
        }

        bool closedGrid(Vec2i coord) const
        {
            return _closed_grid[coord.y*_world_width + coord.x];
        }

        void setClosedGrid(Vec2i coord, bool value)
        {
            _closed_grid[coord.y*_world_width + coord.x] = value;
        }

        void exportPPM(const char* filename, CoordinateList* path);

        enum{
          OBSTACLE = 0,
          EMPTY    = 255
        };
    private:

        HeuristicFunction _heuristic;
        CoordinateList _directions;
        int _world_width;
        int _world_height;
        bool _allow_5x5_search;
        std::vector<uint8_t> _world_grid;
        std::vector<bool> _closed_grid;
        std::vector<uint> _direction_cost;
        std::vector<Node> _memory_storage;
        std::vector<NodePtr> _closed_set;

       // std::multimap<uint, NodePtr> _open_set;
        std::priority_queue<ScoreNodePair, std::vector<ScoreNodePair>, CompareScore> _open_set;

        std::vector<NodePtr> _open_set_2Dmap;
        void  clean();
        NodePtr findMinScoreInOpenSet();

        Node* getNode( NodePtr index) { return &_memory_storage[index]; }
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
