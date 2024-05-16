#pragma once
#include "idiscretisation.h"

namespace cnearest {
/// <summary>
/// Discretisation scheme giving points along a line
/// This has nodes at a startpoint and end point
/// It additionally has n (default 0) midpoint nodes
/// this gives a line with n+2 nodes and n+1 edges
/// </summary>
class LineDiscretisation : public iDiscretisation {
   public:
    LineDiscretisation(const Vector3& start_position,
                              const Vector3& end_position,
                              unsigned int midpoints=0)
        : _start_position(start_position),
          _end_position(end_position),
          _midpoints(midpoints){};
    ~LineDiscretisation() = default;

    Collection<Vector3> positions() const override { 
        Collection<Vector3> result;
        
        //length (inc direction) of an edge
        Vector3 delta = (_end_position - _start_position) / (_midpoints + 1); 

        //2+n nodes starting at start position, seperated by delta
        for (unsigned int i = 0; i < _midpoints+2; i++) {
            result.emplace_back(_start_position + i * delta);
        }
        return result;
    
    }

   private:
    Vector3 _start_position;
    Vector3 _end_position;
    unsigned int _midpoints;
};
}  
