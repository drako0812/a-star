#ifndef ASTAR3_HPP
#define ASTAR3_HPP

#include <queue>
#include <limits>
#include <cmath>

// represents a single pixel
class Node {
public:
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel

    Node(int i, float c) : idx(i),cost(c) {}
};

// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
inline bool operator<(const Node &n1, const Node &n2) {
    return n1.cost > n2.cost;
}

inline bool operator==(const Node &n1, const Node &n2) {
    return n1.idx == n2.idx;
}

// See for various grid heuristics:
// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#S7
// L_\inf norm (diagonal distance)
inline float manhattan(int x0, int y0, int x1, int y1) {
    return 10 * (std::abs(x0 - x1) + std::abs(y0 - y1));
}



// weights:        flattened h x w grid of costs
// h, w:           height and width of grid
// start, goal:    index of start/goal in flattened grid
// diag_ok:        if true, allows diagonal moves (8-conn.)
// paths (output): for each node, stores previous node in path
inline bool astar3(
        const float* weights, const int h, const int w,
        const int start, const int goal,
        int* paths) {

    const float INF = std::numeric_limits<float>::infinity();

    Node start_node(start, 0.);
    Node goal_node(goal, 0.);

    std::vector<float> costs( h * w, INF);
    costs[start] = 0.;

    std::priority_queue<Node> nodes_to_visit;
    nodes_to_visit.push(start_node);

    int nbrs[8];

    int goal_x = goal % w;
    int goal_y = goal / w;

    bool solution_found = false;

    const float travel_cost[8] =
    {
        14,10,14,10,
        10,14,10,14
    };

    while (!nodes_to_visit.empty()) {
        // .top() doesn't actually remove the node
        Node cur = nodes_to_visit.top();

        if (cur == goal_node) {
            solution_found = true;
            break;
        }

        nodes_to_visit.pop();

        int x = cur.idx % w;
        int y = cur.idx / w;
        // check bounds and find up to eight neighbors: top to bottom, left to right
        nbrs[0] = (x > 0 && y > 0)          ? cur.idx - w - 1   : -1;
        nbrs[1] = (x > 0)                   ? cur.idx - 1       : -1;
        nbrs[2] = (x > 0 && y + 1 < h)      ? cur.idx + w - 1   : -1;
        nbrs[3] = (y > 0)                   ? cur.idx - w       : -1;

        nbrs[4] = (y + 1 < h)               ? cur.idx + w       : -1;
        nbrs[5] = (x + 1 < w && y > 0)      ? cur.idx - w + 1   : -1;
        nbrs[6] = (x + 1 < w)               ? cur.idx + 1       : -1;
        nbrs[7] = (x + 1 < w && y + 1 < h ) ? cur.idx + w + 1   : -1;

        float heuristic_cost;
        for (int i=0; i < 8; i++)
        {
            const int index = nbrs[i];
            if (index >= 0 )
            {
                // the sum of the cost so far and the cost of this move
                float new_cost = costs[cur.idx] + weights[index] + travel_cost[i];
                if (new_cost < costs[index])
                {
                    // estimate the cost to the goal based on legal moves

                    heuristic_cost = manhattan(index % w, index / w,
                                               goal_x, goal_y);

                    // paths with lower expected cost are explored first
                    float score = new_cost + heuristic_cost;
                    nodes_to_visit.push(Node(index, score));

                    costs[index] = new_cost;
                    paths[index] = cur.idx;
                }
            }
        }
    }

    return solution_found;
}

#endif // ASTAR3_HPP
