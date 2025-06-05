#ifndef PLANE_H
#define PLANE_H

#include "DataType.h"
#include "Point.h"
#include "Line.h"
#include <vector>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <cmath>

// Forward declarations
template <typename T>
class Plane;

template <typename T>
class Polygon;

// Plane class template
template <typename T = NType>
class Plane {
private:
    Point3D <T>  point_;    // A point on the plane
    Vector3D<T> normal_;    // A normal vector to the plane

public:
    // Constructors
    Plane() : point_(), 
              normal_(Vector3D<T>(static_cast<T>(0), static_cast<T>(0), static_cast<T>(1))) {}
    Plane(const Point3D<T>& point, const Vector3D<T>& normal) : point_(point), normal_(normal.normalized()) {}

    // Distance from a point to the plane
    T distance(const Point3D<T>& p) const;

    // Intersection with a line
    Point3D<T> intersect(const Line<T>& l) const;

    // Containment checks
    bool contains(const Point3D<T>& p) const;
    bool contains(const Line<T>& l) const;

    // Getters
    Point3D<T> getPoint() const { return point_; }
    Vector3D<T> getNormal() const { return normal_; }

    // Setters
    void setPoint(const Point3D<T>& point) { point_ = point; }
    void setNormal(const Vector3D<T>& normal) { normal_ = normal.normalized(); }

    // Operators
    bool operator==(const Plane& other) const;
    bool operator!=(const Plane& other) const { return !(*this == other); }

    // Output operator
    template <typename U>
    friend std::ostream& operator<<(std::ostream& os, const Plane<U>& plane);
};

// Polygon class template
template <typename T = NType>
class Polygon {
private:
    std::vector<Point3D<T>> vertices_;

public:
    // Constructors
    Polygon() : vertices_() {}
    Polygon(const std::vector<Point3D<T>>& vertices) : vertices_(vertices) {}

    // Getters
    std::vector<Point3D<T>> getVertices() const { return vertices_; }
    const Point3D<T>& getVertex(size_t index) const { return vertices_.at(index); }
    Plane<T>    getPlane   () const;    // Get the plane    of the polygon
    Vector3D<T> getNormal  () const;    // Get the normal   of the polygon
    Point3D<T>  getCentroid() const;    // Get the centroid of the polygon

    // Setters
    void setVertices(const std::vector<Point3D<T>>& vertices) { vertices_ = vertices; }

    // Check if a point is inside the polygon
    bool contains(const Point3D<T>& p) const;

    // Get the relation of the polygon with a plane
    RelationType relationWithPlane(const Plane<T>& plane) const;

    // Split the polygon by a plane
    std::pair<Polygon<T>, Polygon<T>> split(const Plane<T>& plane) const;

    // Compute the area of the polygon
    T area() const;

    // Operators
    bool operator==(const Polygon& other) const;
    bool operator!=(const Polygon& other) const { return !(*this == other); }

    // Output operator
    template <typename U>
    friend std::ostream& operator<<(std::ostream& os, const Polygon<U>& polygon);
};


// Equality operators
template <typename T>
bool Plane<T>::operator==(const Plane<T>& other) const {
    bool normalsEqual = (normal_ == other.normal_) || (normal_ == -other.normal_);
    return normalsEqual && contains(other.point_);
}

// Output operator for Plane
template <typename T>
std::ostream& operator<<(std::ostream& os, const Plane<T>& plane) {
    os << "Point: " << plane.point_ << ", Normal: " << plane.normal_;
    return os;
}

// Equality operators
template <typename T>
bool Polygon<T>::operator==(const Polygon<T>& other) const {
    return vertices_ == other.vertices_;
}

// Output operator for Polygon
template <typename T>
std::ostream& operator<<(std::ostream& os, const Polygon<T>& polygon) {
    os << "Vertices: ";
    for (const auto& vertex : polygon.vertices_) {
        os << vertex << " ";
    }
    return os;
}

// ------------------ Polygon ------------------
template <typename T>
Plane<T> Polygon<T>::getPlane() const {
    std::cout << "[DEBUG] getPlane(): vertices = " << vertices_.size() << "\n";

    if (vertices_.size() < 3) {
        std::cerr << "[ERROR] Polygon::getPlane() - menos de 3 vértices ("
                  << vertices_.size() << ")\n";
        throw std::runtime_error("Polygon::getPlane() requiere al menos 3 vértices.");
    }

    const Point3D<T>& p0 = vertices_[0];
    const Point3D<T>& p1 = vertices_[1];
    const Point3D<T>& p2 = vertices_[2];

    Vector3D<T> v1 = p1 - p0;
    Vector3D<T> v2 = p2 - p0;

    std::cout << "[DEBUG] v1: " << v1 << "\n";
    std::cout << "[DEBUG] v2: " << v2 << "\n";
    std::cout << "[DEBUG] v1.cross(v2): " << v1.cross(v2) << "\n";

    Vector3D<T> normal;
    try {
        normal = Vector3D<T>(v1.cross(v2)).unit();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Falló al calcular normal: " << e.what() << "\n";
        throw;
    }

    try {
        normal = Vector3D<T>(v1.cross(v2)).unit();
    } catch (const std::runtime_error& e) {
        std::cerr << "[ERROR] Polygon::getPlane() - No se pudo calcular normal: " << e.what() << "\n";
        throw;
    }

    return Plane<T>(p0, normal);
}


template <typename T>
T Polygon<T>::area() const {
    if (vertices_.size() < 3) return static_cast<T>(0);

    Plane<T> plane = getPlane();
    Vector3D<T> normal = plane.getNormal();
    Vector3D<T> u, v;

    if (std::abs(normal.getX().getValue()) < 0.9f)
        u = normal.cross(Vector3D<T>(1, 0, 0)).normalized();
    else
        u = normal.cross(Vector3D<T>(0, 1, 0)).normalized();
    v = normal.cross(u);

    T area = static_cast<T>(0);
    for (size_t i = 0; i < vertices_.size(); ++i) {
        const auto& p1 = vertices_[i];
        const auto& p2 = vertices_[(i + 1) % vertices_.size()];
        T x1 = u.dot(p1 - vertices_[0]);
        T y1 = v.dot(p1 - vertices_[0]);
        T x2 = u.dot(p2 - vertices_[0]);
        T y2 = v.dot(p2 - vertices_[0]);
        area += (x1 * y2 - x2 * y1);
    }
    return abs(area) * static_cast<T>(0.5);
}

template <typename T>
bool Polygon<T>::contains(const Point3D<T>& p) const {
    Plane<T> plane = getPlane();
    if (std::abs(plane.distance(p).getValue()) > 1e-3f)
        return false;

    Vector3D<T> normal = plane.getNormal();
    Vector3D<T> u, v;
    if (std::abs(normal.getX().getValue()) < 0.9f)
        u = normal.cross(Vector3D<T>(1, 0, 0)).normalized();
    else
        u = normal.cross(Vector3D<T>(0, 1, 0)).normalized();
    v = normal.cross(u);

    std::vector<std::pair<T, T>> proj;
    for (const auto& vert : vertices_) {
        T x = u.dot(vert - vertices_[0]);
        T y = v.dot(vert - vertices_[0]);
        proj.emplace_back(x, y);
    }

    T px = u.dot(p - vertices_[0]);
    T py = v.dot(p - vertices_[0]);

    bool inside = false;
    for (size_t i = 0, j = proj.size() - 1; i < proj.size(); j = i++) {
        auto [xi, yi] = proj[i];
        auto [xj, yj] = proj[j];
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi + 1e-6f) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

// ------------------ Plane ------------------
template <typename T>
T Plane<T>::distance(const Point3D<T>& p) const {
    return normal_.dot(p - point_);
}

template <typename T>
Point3D<T> Plane<T>::intersect(const Line<T>& l) const {
    T denom = normal_.dot(l.getDirection());
    if (std::abs(denom.getValue()) < 1e-6f)
        throw std::runtime_error("Line is parallel to the plane");
    T t = normal_.dot(point_ - l.getPoint()) / denom;
    return l.getPoint() + l.getDirection() * t;
}

template <typename T>
bool Plane<T>::contains(const Point3D<T>& p) const {
    return std::abs(distance(p).getValue()) < 1e-3f;
}

template <typename T>
bool Plane<T>::contains(const Line<T>& l) const {
    return contains(l.getPoint()) && std::abs(normal_.dot(l.getDirection()).getValue()) < 1e-6f;
}

template <typename T>
RelationType Polygon<T>::relationWithPlane(const Plane<T>& plane) const {
    int inFront = 0, behind = 0;

    for (const auto& v : vertices_) {
        T d = plane.distance(v);
        if (d > T(1e-4)) inFront++;
        else if (d < T(-1e-4)) behind++;
    }

    if (inFront > 0 && behind > 0) return SPLIT;
    if (inFront > 0) return IN_FRONT;
    if (behind > 0) return BEHIND;
    return COINCIDENT;
}

template <typename T>
std::pair<Polygon<T>, Polygon<T>> Polygon<T>::split(const Plane<T>& plane) const {
    std::vector<Point3D<T>> front, back;

    for (size_t i = 0; i < vertices_.size(); ++i) {
        const auto& current = vertices_[i];
        const auto& next = vertices_[(i + 1) % vertices_.size()];
        T d1 = plane.distance(current);
        T d2 = plane.distance(next);

        bool cFront = d1 >= T(0);
        bool nFront = d2 >= T(0);

        if (cFront) front.push_back(current);
        else back.push_back(current);

        if ((cFront && !nFront) || (!cFront && nFront)) {
            // Compute intersection point
            T t = d1 / (d1 - d2);
            Point3D<T> intersect = current + (next - current) * t;
            front.push_back(intersect);
            back.push_back(intersect);
        }
    }

    return { Polygon<T>(front), Polygon<T>(back) };
}

#endif // PLANE_H