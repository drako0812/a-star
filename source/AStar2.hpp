/*
    Copyright (c) 2018, Eurecat
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef _ASTAR2_Taffete_eurecat_
#define _ASTAR2_Taffete_eurecat_

#include <vector>
#include <functional>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>

namespace AStar
{

struct Coord2D
{
    int x, y;
    bool operator == (const Coord2D& coordinates);
};


using HeuristicFunction = std::function<float(Coord2D, Coord2D)>;
using CoordinateList = std::vector<Coord2D>;


typedef std::pair<float,Coord2D> ScoreCoordPair;

struct CompareScore
{
    //Note: we want the priority_queue to be ordered from smaller to larger
    bool operator() (const ScoreCoordPair& a,
                     const ScoreCoordPair& b)
    {
        return a.first > b.first;
    }
};

class PathFinder
{

public:
    PathFinder();
    ~PathFinder();

    /// Row-major ordered map, where an obstacle is represented as a pixel with value 0 (black)
    void setWorldData(int width, int height, const uint8_t *data);

    /// Default value is Heuristic::manhattan
    void setHeuristic(HeuristicFunction heuristic_);

    /// Function that performs the actual A* computation.
    CoordinateList  findPath(Coord2D source_, Coord2D target_);


    /// If false, it looks at the neighbours ina 3x3 area arounf the current position.
    /// If false, a 5x5 search area is used instead.
    void allow5by5(bool allow)
    {
        _allow_5x5_search = allow;
    }

    /// Export the resulting solution in a visual way. Useful for debugging.
    void exportPPM(const char* filename, CoordinateList* path);

    enum{
        OBSTACLE = 0,
        EMPTY    = 255
    };

    struct Cell{
        uint8_t  world;
        bool     already_visited;
        uint32_t path_parent_index;
        float    cost_G;
    };

    const Cell& cell(Coord2D coordinates_) const
    {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }

    Cell& cell(Coord2D coordinates_)
    {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }

private:

    HeuristicFunction _heuristic;
    int _world_width;
    int _world_height;
    bool _allow_5x5_search;

    std::array<Coord2D,24> _directions;
    std::array<float,24>   _direction_cost;

    std::priority_queue<ScoreCoordPair, std::vector<ScoreCoordPair>, CompareScore> _open_set;

    bool detectCollision(Coord2D coordinates);

    std::vector<Cell> _gridmap;

    void  clean();
};

class Heuristic
{
    static Coord2D getDelta(Coord2D source_, Coord2D target_);

public:
    static float manhattan(Coord2D source_, Coord2D target_);
    static float euclidean(Coord2D source_, Coord2D target_);
    static float octagonal(Coord2D source_, Coord2D target_);
};

}

#endif
