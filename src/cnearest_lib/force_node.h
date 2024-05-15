#pragma once
#include "eigen.h"

class ForceNode {
    ForceNode();
    ~ForceNode();

    Eigen::Vector3d position();
    Eigen::Vector3d velocity();
};