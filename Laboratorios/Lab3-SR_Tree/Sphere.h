#ifndef SPHERE_H
#define SPHERE_H

#include "Point.h"

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

void Sphere::expandToInclude(const Sphere& other) {// Expande la esfera para incluir otra esfera
    float dist_c_c = Point::distance(center, other.center);
    cout << "Distancia entre centros: " << dist_c_c << std::endl;

    if (dist_c_c + other.radius <= radius + EPSILON) return; // La otra esfera ya está dentro

    if (dist_c_c + radius <= other.radius + EPSILON) { // La esfera actual está dentro de la otra
        // cout<<"estoy dentro de la otra esfera"<<std::endl;
        center = other.center;
        radius = other.radius;
        return;
    }

    float R_new = (dist_c_c + radius + other.radius) / 2.0f;
    Point direc = (other.center - center);

    center = center + (direc/direc.norm()) * (R_new - radius);
    radius = R_new;
}

void Sphere::expandToInclude(const Point& p){// Expande la esfera para incluir un punto
    float dist_c_p = Point::distance(center, p);
    if (dist_c_p <= radius + EPSILON) return; // El punto ya está dentro

    radius = dist_c_p; // Si el punto está fuera, simplemente aumentamos el radio
}

#endif // SPHERE_H
