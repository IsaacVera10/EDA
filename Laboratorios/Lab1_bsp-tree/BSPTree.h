#ifndef BSPTREE_H
#define BSPTREE_H

#include <memory>
#include <vector>
#include <functional>
#include "Plane.h"
#include "Ball.h"

// Forward declarations
template <typename T>
class BSPNode;

template <typename T>
class BSPTree;

// BSPNode class template
template <typename T = NType>
class BSPNode {
private:
    Plane<T> partition_;
    std::unique_ptr<BSPNode<T>> front_;
    std::unique_ptr<BSPNode<T>> back_;
    std::vector<Polygon<T>> polygons_;

public:
    BSPNode() : partition_(), front_(nullptr), back_(nullptr) {}
    ~BSPNode() = default;

    BSPNode(const BSPNode&) = delete;
    BSPNode& operator=(const BSPNode&) = delete;

    // Getters
    const Plane<T>& getPartition() const { return partition_; }
    const std::vector<Polygon<T>>& getPolygons() const { return polygons_; }
    const BSPNode<T>* getFront() const { return front_.get(); }
    const BSPNode<T>*  getBack() const { return  back_.get(); }

    void insert(const Polygon<T>& polygon);

    // Método de consulta: recolecta en 'results' los polígonos que pueden colisionar con la Ball.
    void query(const Ball<T>& ball, const LineSegment<T>& movement, std::vector<Polygon<T>>& results) const;
    
    // Print
    void print(std::ostream& os, int indent = 0) const{
        std::string indentStr(indent * 4, ' ');
        
        os << indentStr << "BSPNode:\n";
        os << indentStr << "  Partition: " << partition_ << "\n";
        
        // Imprimir  polígonos
        os << indentStr << "  Polygons (" << polygons_.size() << "): ";
        if (polygons_.empty()) {
            os << "None\n";
        } else {
            os << "\n";
            for (size_t i = 0; i < polygons_.size(); ++i) {
                os << indentStr << "    [" << i << "]: " << polygons_[i] << "\n";
            }
        }
        
        // Front
        if (front_) {
            os << indentStr << "  Front:\n";
            front_->print(os, indent + 1);
        } else {
            os << indentStr << "  Front: NULL\n";
        }
        
        // Back
        if (back_) {
            os << indentStr << "  Back:\n";
            back_->print(os, indent + 1);
        } else {
            os << indentStr << "  Back: NULL\n";
        }
    }


    // Recorrido
    void collectNodes(std::vector<const BSPNode<T>*>& nodes) const{
        nodes.push_back(this);
        if (front_)
            front_->collectNodes(nodes);
        if (back_)
            back_->collectNodes(nodes);
    }

    void collectPolygons(std::vector<Polygon<T>>& polys) const {
        polys.insert(polys.end(), polygons_.begin(), polygons_.end());
        if (front_)
            front_->collectPolygons(polys);
        if (back_)
            back_->collectPolygons(polys);
    }

    void traverse(std::function<void(const BSPNode<T>&)> func) const {
        func(*this);
        if (front_)
            front_->traverse(func);
        if (back_)
            back_->traverse(func);
    }
};

// BSPTree class template
template <typename T = NType>
class BSPTree {
private:
    std::unique_ptr<BSPNode<T>> root_;

public:
    BSPTree() : root_(nullptr) {}
    ~BSPTree() = default;

    void insert(const Polygon<T>& polygon);
    
    // Devuelve los polígonos candidatos a colisión con la Ball.
    std::vector<Polygon<T>> query(const Ball<T>& ball, const LineSegment<T>& movement) const;
    
    // Print
    void print(std::ostream& os) const{
        if (root_) {
            os << "BSPTree:\n";
            root_->print(os, 1); 
        } else {
            os << "BSPTree is empty.\n";
        }
    }


    // Funciones de recorrido
    std::vector<const BSPNode<T>*> getAllNodes() const{
        std::vector<const BSPNode<T>*> nodes;
        if (root_) {
            root_->collectNodes(nodes);
        } else {//Verficar si el árbol está vacío
            std::cerr << "[WARN] Árbol vacío: root_ es nullptr.\n";
        }
        return nodes;
    }


    std::vector<Polygon<T>> getAllPolygons() const{
        std::vector<Polygon<T>> polys;
        if (root_)
            root_->collectPolygons(polys);
        return polys;
    }
    void traverse(std::function<void(const BSPNode<T>&)> func) const{
        if (root_)
            root_->traverse(func);
    }
};

template <typename T>
void BSPNode<T>::insert(const Polygon<T>& polygon) {
    if (polygons_.empty()) {
        partition_ = polygon.getPlane();
        polygons_.push_back(polygon);
        return;
    }
    RelationType rel = polygon.relationWithPlane(partition_);
    switch (rel) {
        case COINCIDENT:
            polygons_.push_back(polygon);
            break;
    
        case IN_FRONT:
            if (!front_) front_ = std::unique_ptr<BSPNode<T>>(new BSPNode<T>());
            front_->insert(polygon);
            break;
    
        case BEHIND:
            if (!back_) back_ = std::unique_ptr<BSPNode<T>>(new BSPNode<T>());
            back_->insert(polygon);
            break;
    
        case SPLIT: {
            auto splitResult = polygon.split(partition_);
            if (!front_) front_ = std::unique_ptr<BSPNode<T>>(new BSPNode<T>());
            front_->insert(splitResult.first);
            if (!back_) back_ = std::unique_ptr<BSPNode<T>>(new BSPNode<T>());
            back_->insert(splitResult.second);
            break;
        }
    
        default:
            throw std::logic_error("Tipo de relación desconocida en BSPNode::insert");
    }
}


template <typename T>
void BSPNode<T>::query(const Ball<T>& ball, const LineSegment<T>& movement, std::vector<Polygon<T>>& results) const {
    T r = ball.getRadius();
    T d1 = partition_.distance(movement.getP1());
    T d2 = partition_.distance(movement.getP2());

    bool startInFront = d1 > -r;
    bool endInFront = d2 > -r;
    bool startBehind = d1 < r;
    bool endBehind = d2 < r;

    if (startInFront || endInFront) {
        if (front_)
            front_->query(ball, movement, results);
    }
    if (startBehind || endBehind) {
        if (back_)
            back_->query(ball, movement, results);
    }

    for (const auto& poly : polygons_) {
        Plane<T> plane = poly.getPlane();
        Vector3D<T> dir = movement.getP2() - movement.getP1();
        T denom = plane.getNormal().dot(dir);

        if (abs(denom) > T(1e-5)) {
            T t = (plane.getPoint() - movement.getP1()).dot(plane.getNormal()) / denom;
            if (t >= T(0) && t <= T(1)) {
                Point3D<T> intersection = movement.getP1() + dir * t;
                if (poly.contains(intersection)) {
                    results.push_back(poly);
                }
            }
        } else {
            if (poly.contains(ball.getPosition()) || poly.contains(movement.getP2())) {
                results.push_back(poly);
            }
        }
    }
}

// ------------------ BSPTree ------------------
template <typename T>
void BSPTree<T>::insert(const Polygon<T>& polygon) {
    if (!root_) root_ = std::unique_ptr<BSPNode<T>>(new BSPNode<T>());
    root_->insert(polygon);
}

template <typename T>
std::vector<Polygon<T>> BSPTree<T>::query(const Ball<T>& ball, const LineSegment<T>& movement) const {
    std::vector<Polygon<T>> results;
    if (root_) root_->query(ball, movement, results);
    return results;
}





#endif // BSPTREE_H
