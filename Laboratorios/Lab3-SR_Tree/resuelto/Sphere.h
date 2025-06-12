#ifndef SPHERE_H
#define SPHERE_H

#include "Point.h"
#include <cmath>

struct Sphere {
    Point center;
    float radius;

    Sphere() 
      : center(), radius(0.0f) {}

    Sphere(const Point& c, float r)
      : center(c), radius(r) {}

    void expandToInclude(const Sphere& other);
    void expandToInclude(const Point& p);
};

void Sphere::expandToInclude(const Sphere& other) {
    if (radius == 0.0f) {
        center = other.center;
        radius = other.radius;
        return;
    }
    
    float dist = Point::distance(center, other.center);
    if (dist + other.radius <= radius + 1e-6f) {
        // La otra esfera está completamente contenida
        return;
    }
    if (dist + radius <= other.radius + 1e-6f) {
        // Esta esfera está completamente contenida en la otra
        center = other.center;
        radius = other.radius;
        return;
    }
    
    // Necesitamos crear una nueva esfera que contenga ambas
    float newRadius = (radius + other.radius + dist) / 2.0f;
    if (dist > 1e-6f) {
        float t = (newRadius - radius) / dist;
        Point newCenter = center + (other.center - center) * t;
        center = newCenter;    }
    radius = newRadius;
}

void Sphere::expandToInclude(const Point& p) {
    if (radius == 0.0f) {
        float dist = Point::distance(center, p);
        if (dist <= 1e-6f) {
            // Es el mismo punto, mantener radio 0
            return;
        } else {
            // Expandir para incluir el nuevo punto
            radius = dist / 2.0f;
            center = center + (p - center) * 0.5f;
            return;
        }
    }
    
    float dist = Point::distance(center, p);
    if (dist <= radius + 1e-6f) {
        // El punto ya está contenido
        return;
    }
    
    // Expandir la esfera para incluir el punto
    float newRadius = (radius + dist) / 2.0f;
    if (dist > 1e-6f) {
        float t = (newRadius - radius) / dist;
        Point newCenter = center + (p - center) * t;
        center = newCenter;
    }
    radius = newRadius;
}

#endif // SPHERE_H
