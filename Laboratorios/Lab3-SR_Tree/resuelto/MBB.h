#ifndef MBB_H
#define MBB_H

#include "Point.h"
#include <algorithm>
#include <cmath>
#include <utility>

struct MBB {
    Point minCorner, maxCorner;

    MBB() 
      : minCorner(), maxCorner() {}    MBB(const Point& p) 
      : minCorner(p), maxCorner(p) {}
    MBB(Point min, Point max)
      : minCorner(min), maxCorner(max) {}
    template<typename T1, typename T2>
    MBB(T1&& min, T2&& max)
      : minCorner(std::forward<T1>(min)), maxCorner(std::forward<T2>(max)) {}
    MBB(const MBB& other)
        : minCorner(other.minCorner), maxCorner(other.maxCorner) {}

    void expandToInclude(const MBB& other);
    void expandToInclude(const Point& p);
    static float maxDist(const Point& p, const MBB& box);
    static MBB createFromPoints(const Point& min, const Point& max);

};

void MBB::expandToInclude(const MBB& other) {
    for (std::size_t i = 0; i < DIM; ++i) {
        minCorner[i] = std::min(minCorner[i], other.minCorner[i]);
        maxCorner[i] = std::max(maxCorner[i], other.maxCorner[i]);
    }
}

void MBB::expandToInclude(const Point& p) {
    for (std::size_t i = 0; i < DIM; ++i) {
        minCorner[i] = std::min(minCorner[i], p[i]);
        maxCorner[i] = std::max(maxCorner[i], p[i]);
    }
}

float MBB::maxDist(const Point& p, const MBB& box) {
    float maxDistSq = 0.0f;
    for (std::size_t i = 0; i < DIM; ++i) {
        float coord = p[i];
        float distToMin = std::abs(coord - box.minCorner[i]);
        float distToMax = std::abs(coord - box.maxCorner[i]);
        float maxDistAxis = std::max(distToMin, distToMax);
        maxDistSq += maxDistAxis * maxDistAxis;
    }
    return std::sqrt(maxDistSq);
}

MBB MBB::createFromPoints(const Point& min, const Point& max) {
    return MBB(min, max);
}

// Helper function to avoid most vexing parse
inline MBB makeMBB(const Point& min, const Point& max) {
    return MBB(min, max);
}

#endif // MBB_H
