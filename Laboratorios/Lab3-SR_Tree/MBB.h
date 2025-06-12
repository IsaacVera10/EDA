#ifndef MBB_H
#define MBB_H
using namespace std;
#include "Point.h"

struct MBB {
    Point minCorner, maxCorner;

    MBB() 
      : minCorner(), maxCorner() {}
    explicit MBB(const Point& p) 
      : minCorner(p), maxCorner(p) {}
    MBB(const Point& min, const Point& max)
      : minCorner(min), maxCorner(max) {}
    MBB(const MBB& other)
        : minCorner(other.minCorner), maxCorner(other.maxCorner) {}

    void expandToInclude(const MBB& other);
    void expandToInclude(const Point& p);
    static float maxDist(const Point& p, const MBB& box);
    bool intersecta(const MBB& other) const;
    bool contiene(const Point& p) const;
};

void MBB::expandToInclude(const MBB& other) {
// Para cada dimensión, voy comparando y eligiendo el mínimo y máximo corner
// Es una función muy determinísta(no varía mucho de otras implementaciones)
// Las otras implementaciones podría ser usar if u operadores ternarios
    for (size_t j = 0; j < DIM; j++) {
        minCorner[j] = std::min(minCorner[j], other.minCorner[j]);
        maxCorner[j] = std::max(maxCorner[j], other.maxCorner[j]);
    }
}

void MBB::expandToInclude(const Point& p) {
// Para cada dimensión, voy comparando y eligiendo el mínimo y máximo corner
// Es una función muy determinísta,(no varía mucho de otras implementaciones)
// Las otras implementaciones podría ser usar if u operadores ternarios
    for (size_t j = 0; j < DIM; j++) {
        minCorner[j] = std::min(minCorner[j], p[j]);
        maxCorner[j] = std::max(maxCorner[j], p[j]);
    }
}
float MBB::maxDist(const Point& p, const MBB& box) {
//Dado un punto centro p y una caja MBB,
//Calcula el radio que debe tener una esfera que contenga la caja
//Para cada dimenión:
//  d1 = distancia al corner mínimo
//  d2 = distancia al corner máximo
//  suma+= max(d1, d2)^2
    float suma = 0.0f;
    for (size_t i = 0; i < DIM; i++) {
      float d1 = std::abs(p[i] - box.minCorner[i]);
      float d2 = std::abs(p[i] - box.maxCorner[i]);
      suma += pow(std::max(d1, d2), 2); // Sumo el cuadrado de la distancia máxima
    }
    return std::sqrt(suma);
}

bool MBB::intersecta(const MBB& other) const {
    for (size_t j = 0; j < DIM; j++) if (maxCorner[j] < other.minCorner[j] || minCorner[j] > other.maxCorner[j]) return false;
    return true;
}

bool MBB::contiene(const Point& p) const {
    for (std::size_t j = 0; j < DIM; j++) if (p[j] < minCorner[j]-EPSILON || p[j] > maxCorner[j] + EPSILON) return false;
    return true;
}
#endif // MBB_H
