// Archivo: Plane.h + Polygon
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
    Point3D<T> point_;
    Vector3D<T> normal_;

public:
    Plane() : point_(), normal_(Vector3D<T>(static_cast<T>(0), static_cast<T>(0), static_cast<T>(1))) {}
    Plane(const Point3D<T>& point, const Vector3D<T>& normal) : point_(point), normal_(normal.normalized()) {}

    T distance(const Point3D<T>& p) const {
        return normal_.dot(p - point_);
    }

    Point3D<T> intersect(const Line<T>& l) const {
        T denom = normal_.dot(l.getDirection());
        if (denom == static_cast<T>(0)) {
            throw std::runtime_error("La línea es paralela al plano.");
        }
        T t = normal_.dot(point_ - l.getPoint()) / denom;
        return l.getPoint() + l.getDirection() * t;
    }

    bool contains(const Point3D<T>& p) const {
        return abs(distance(p)) < static_cast<T>(1e-3);
    }

    bool contains(const Line<T>& l) const {
        return contains(l.getPoint()) && isOrthogonal(normal_, l.getDirection());
    }

    Point3D<T> getPoint() const { return point_; }
    Vector3D<T> getNormal() const { return normal_; }

    void setPoint(const Point3D<T>& point) { point_ = point; }
    void setNormal(const Vector3D<T>& normal) { normal_ = normal.normalized(); }

    bool operator==(const Plane& other) const {
        bool normalsEqual = (normal_ == other.normal_) || (normal_ == -other.normal_);
        return normalsEqual && contains(other.point_);
    }
    bool operator!=(const Plane& other) const { return !(*this == other); }

    template <typename U>
    friend std::ostream& operator<<(std::ostream& os, const Plane<U>& plane);
};

// Polygon class template
template <typename T = NType>
class Polygon {
private:
    std::vector<Point3D<T>> vertices_;

public:
    Polygon() : vertices_() {}
    Polygon(const std::vector<Point3D<T>>& vertices) : vertices_(vertices) {}

    std::vector<Point3D<T>> getVertices() const { return vertices_; }
    const Point3D<T>& getVertex(size_t index) const { return vertices_.at(index); }
    Plane<T> getPlane() const {
        if (vertices_.size() < 3)
            throw std::runtime_error("No se puede definir un plano con menos de 3 vértices.");
        Vector3D<T> normal = (vertices_[1] - vertices_[0]).cross(vertices_[2] - vertices_[0]).normalized();
        return Plane<T>(vertices_[0], normal);
    }
    Vector3D<T> getNormal() const {
        return getPlane().getNormal();
    }
    Point3D<T> getCentroid() const {
        Point3D<T> sum;
        for (const auto& v : vertices_) sum += v;
        return sum / static_cast<T>(vertices_.size());
    }

    void setVertices(const std::vector<Point3D<T>>& vertices) { vertices_ = vertices; }

    bool contains(const Point3D<T>& p) const {
        Plane<T> plane = getPlane();
        if (!plane.contains(p)) return false;

        size_t n = vertices_.size();
        for (size_t i = 0; i < n; ++i) {
            Point3D<T> a = vertices_[i];
            Point3D<T> b = vertices_[(i + 1) % n];
            Vector3D<T> edge = b - a;
            Vector3D<T> toPoint = p - a;
            Vector3D<T> cross = edge.cross(toPoint);
            if (cross.dot(getNormal()) < static_cast<T>(0))
                return false;
        }
        return true;
    }

    RelationType relationWithPlane(const Plane<T>& plane) const {
        bool inFront = false, behind = false;
        for (const auto& v : vertices_) {
            T d = plane.distance(v);
            if (d > T(1e-3)) inFront = true;
            else if (d < T(-1e-3)) behind = true;
        }
        if (inFront && behind) return SPLIT;
        else if (inFront) return IN_FRONT;
        else if (behind) return BEHIND;
        return COINCIDENT;
    }

    std::pair<Polygon<T>, Polygon<T>> split(const Plane<T>& plane) const {
        std::vector<Point3D<T>> frontVerts, backVerts;
        size_t n = vertices_.size();

        for (size_t i = 0; i < n; ++i) {
            const auto& current = vertices_[i];
            const auto& next = vertices_[(i + 1) % n];
            T d1 = plane.distance(current);
            T d2 = plane.distance(next);

            bool currInFront = d1 > T(1e-3);
            bool nextInFront = d2 > T(1e-3);

            if (currInFront) frontVerts.push_back(current);
            else backVerts.push_back(current);

            if ((currInFront && !nextInFront) || (!currInFront && nextInFront)) {
                T t = d1 / (d1 - d2);
                Point3D<T> intersect = current + (next - current) * t;
                frontVerts.push_back(intersect);
                backVerts.push_back(intersect);
            }
        }

        return {Polygon<T>(frontVerts), Polygon<T>(backVerts)};
    }

    T area() const {
        if (vertices_.size() < 3) return T(0);
        Vector3D<T> normal = getNormal();
        T areaSum = 0;
        for (size_t i = 0; i < vertices_.size(); ++i) {
            Point3D<T> p1 = vertices_[i];
            Point3D<T> p2 = vertices_[(i + 1) % vertices_.size()];
            Vector3D<T> cross = p1.cross(p2);
            areaSum += normal.dot(cross);
        }
        return abs(areaSum) / static_cast<T>(2);
    }

    bool operator==(const Polygon& other) const {
        return vertices_ == other.vertices_;
    }
    bool operator!=(const Polygon& other) const { return !(*this == other); }

    template <typename U>
    friend std::ostream& operator<<(std::ostream& os, const Polygon<U>& polygon);
};

// Output operators

template <typename T>
std::ostream& operator<<(std::ostream& os, const Plane<T>& plane) {
    os << "Point: " << plane.point_ << ", Normal: " << plane.normal_;
    return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Polygon<T>& polygon) {
    os << "Vertices: ";
    for (const auto& vertex : polygon.getVertices()) {
        os << vertex << " ";
    }
    return os;
}

#endif // PLANE_H