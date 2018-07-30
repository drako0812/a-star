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

    struct Node
    {
        float G;
        int16_t coord_x, coord_y;

        Node(Vec2i coord = {0,0});

        Vec2i coordinates() const
        {
            return {coord_x, coord_y};
        }
    };

    typedef std::pair<uint,Node> ScoreNodePair;

    struct CompareScore
    {
      bool operator() (const ScoreNodePair& a,
                       const ScoreNodePair& b)
        {
          return a.first > b.first;
        }
    };

    class Generator
    {
        bool detectCollision(Vec2i coordinates);

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

        void allow5by5(bool allow)
        {
            _allow_5x5_search = allow;
        }

        uint8_t& worldGrid(Vec2i coordinates_)
        {
            return _world_grid[coordinates_.y*_world_width + coordinates_.x];
        }

        void exportPPM(const char* filename, CoordinateList* path);

        enum{
          OBSTACLE = 0,
          EMPTY    = 255
        };
    private:

        HeuristicFunction _heuristic;
        CoordinateList    _directions;
        std::vector<uint> _direction_cost;
        int _world_width;
        int _world_height;
        bool _allow_5x5_search;
        std::vector<uint8_t> _world_grid;
        std::vector<bool>    _closed_grid;

        std::priority_queue<ScoreNodePair, std::vector<ScoreNodePair>, CompareScore> _open_set;

        std::vector<float>    _cost_map;
        std::vector<uint32_t> _path_map;

        void  clean();
        Node findMinScoreInOpenSet();
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
