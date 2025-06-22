#ifndef ARENA_POINTS_CPP
#define ARENA_POINTS_CPP

struct ArenaPoint{
    Eigen::Vector3d coordinates;
    bool is_visited = false;
};

#endif