#ifndef SRTREE_H
#define SRTREE_H

#include <vector>
#include <limits>
#include <algorithm>
#include <string>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <queue>
#include "Point.h"
#include "MBB.h"
#include "Sphere.h"


class SRNode {
private:
    MBB       _boundingBox;
    Sphere _boundingSphere;
    SRNode*        _parent;
    std::vector<Point*>    _points;
    std::vector<SRNode*> _children;    bool _isLeaf;

public:
    SRNode() : _parent(nullptr), _isLeaf(true) {}
    
    bool    getIsLeaf() const { return _isLeaf; }
    SRNode* getParent() const { return _parent; }

    const    MBB& getBoundingBox   () const { return    _boundingBox; }
    const Sphere& getBoundingSphere() const { return _boundingSphere; }
    const std::vector< Point*>&   getPoints() const { return   _points; }
    const std::vector<SRNode*>& getChildren() const { return _children; }    std::size_t getNumPoints  () const { return   _points.size(); }
    std::size_t getNumChildren() const { return _children.size(); }    void setBoundingBox   (const    MBB&    box) { _boundingBox    =    box; }
    void setBoundingSphere(const Sphere& sphere) { _boundingSphere = sphere; }
    void setParent        (      SRNode* parent) {         _parent = parent; }
    void setIsLeaf        (bool          isLeaf) {         _isLeaf = isLeaf; }
      void addChild(SRNode* child) { _children.push_back(child); }
    void addPoint(Point* point) { _points.push_back(point); }
    void resizePoints(std::size_t size) { _points.resize(size); }
    void resizeChildren(std::size_t size) { _children.resize(size); }// Insert algorithm
    SRNode* insert(Point& _data, std::size_t maxEntries);
    
    // Auxiliary methods
    SRNode* split(std::size_t maxEntries);
    SRNode* chooseBestChild(const Point& point);
    float calculateVolumeIncrease(SRNode* node, const Point& point);
    float calculateVolume(const MBB& box);
    void updateBoundingVolumes();
};


class SRTree {
private:
    SRNode*           _root;
    std::size_t _maxEntries;
    
public:

    SRTree() : _maxEntries(15), _root(nullptr) {}
    explicit SRTree(std::size_t maxEntries) : _maxEntries(maxEntries), _root(nullptr) {}

    SRNode* getRoot() const { return _root; }

    void insert(const Point& point);
    bool search(const Point& point) const;
    std::vector<Point*> rangeQuery(const MBB& box) const;
    std::vector<Point*> rangeQuery(const Sphere& sphere) const;

    // k-nearest neighbors search
    std::vector<Point*> kNearestNeighbors(const Point& point, std::size_t k) const;

private:
    // Auxiliary methods
    bool searchRecursive(SRNode* node, const Point& point) const;
    bool pointIntersectsMBB(const Point& point, const MBB& box) const;
    bool pointIntersectsSphere(const Point& point, const Sphere& sphere) const;
    bool mbbIntersectsMBB(const MBB& box1, const MBB& box2) const;
    bool sphereIntersectsMBB(const Sphere& sphere, const MBB& box) const;
    void rangeQueryMBBRecursive(SRNode* node, const MBB& box, std::vector<Point*>& result) const;
    void rangeQuerySphereRecursive(SRNode* node, const Sphere& sphere, std::vector<Point*>& result) const;
    void kNNRecursive(SRNode* node, const Point& point, std::vector<std::pair<float, Point*>>& candidates) const;
    float calculateMinDistToMBB(const Point& point, const MBB& box) const;
};

#endif // SRTREE_H

// ============== IMPLEMENTACIÓN DE SRNode ==============

SRNode* SRNode::insert(Point& data, std::size_t maxEntries) {    
    if (_isLeaf) {
        addPoint(&data);
        
        // Actualizar bounding volumes
        if (_points.size() == 1) {
            _boundingBox = MBB(data);
            _boundingSphere = Sphere(data, 0.0f);
        } else {
            _boundingBox.expandToInclude(data);
            _boundingSphere.expandToInclude(data);
        }
        
        // Verificar si necesita split
        if (_points.size() > maxEntries) {
            return split(maxEntries);
        }
        return nullptr;
    } else {
        // Encontrar el mejor hijo para insertar
        SRNode* bestChild = chooseBestChild(data);
        SRNode* newNode = bestChild->insert(data, maxEntries);
        
        // Actualizar bounding volumes
        updateBoundingVolumes();
          if (newNode) {
            addChild(newNode);
            newNode->setParent(this);
            
            if (_children.size() > maxEntries) {
                return split(maxEntries);
            }
        }
        return nullptr;
    }
}

SRNode* SRNode::split(std::size_t maxEntries) {
    SRNode* newNode = new SRNode();
    newNode->setIsLeaf(_isLeaf);
    newNode->setParent(_parent);
      if (_isLeaf) {
        // Split de nodo hoja
        std::size_t mid = _points.size() / 2;
        for (std::size_t i = mid; i < _points.size(); ++i) {
            newNode->addPoint(_points[i]);
        }
        resizePoints(mid);
    } else {
        // Split de nodo interno
        std::size_t mid = _children.size() / 2;
        for (std::size_t i = mid; i < _children.size(); ++i) {
            newNode->addChild(_children[i]);
            _children[i]->setParent(newNode);
        }
        resizeChildren(mid);
    }
    
    updateBoundingVolumes();
    newNode->updateBoundingVolumes();
    
    return newNode;
}

SRNode* SRNode::chooseBestChild(const Point& point) {
    SRNode* best = _children[0];
    float minIncrease = calculateVolumeIncrease(best, point);
    
    for (std::size_t i = 1; i < _children.size(); ++i) {
        float increase = calculateVolumeIncrease(_children[i], point);
        if (increase < minIncrease) {
            minIncrease = increase;
            best = _children[i];
        }
    }
    return best;
}

float SRNode::calculateVolumeIncrease(SRNode* node, const Point& point) {
    MBB oldBox = node->_boundingBox;
    MBB newBox = oldBox;
    newBox.expandToInclude(point);
    
    float oldVolume = calculateVolume(oldBox);
    float newVolume = calculateVolume(newBox);
    
    return newVolume - oldVolume;
}

float SRNode::calculateVolume(const MBB& box) {
    float volume = 1.0f;
    for (std::size_t i = 0; i < DIM; ++i) {
        volume *= (box.maxCorner[i] - box.minCorner[i]);
    }
    return volume;
}

void SRNode::updateBoundingVolumes() {
    if (_isLeaf) {
        if (!_points.empty()) {
            _boundingBox = MBB(*_points[0]);
            _boundingSphere = Sphere(*_points[0], 0.0f);
            
            for (std::size_t i = 1; i < _points.size(); ++i) {
                _boundingBox.expandToInclude(*_points[i]);
                _boundingSphere.expandToInclude(*_points[i]);
            }
        }
    } else {
        if (!_children.empty()) {
            _boundingBox = _children[0]->_boundingBox;
            _boundingSphere = _children[0]->_boundingSphere;
            
            for (std::size_t i = 1; i < _children.size(); ++i) {
                _boundingBox.expandToInclude(_children[i]->_boundingBox);
                _boundingSphere.expandToInclude(_children[i]->_boundingSphere);
            }
        }
    }
}

// ============== IMPLEMENTACIÓN DE SRTree ==============

void SRTree::insert(const Point& point) {
    Point* pointPtr = new Point(point);
    
    if (!_root) {
        _root = new SRNode();
        _root->setIsLeaf(true);
        _root->setParent(nullptr);
    }
    
    SRNode* newNode = _root->insert(*pointPtr, _maxEntries);
    
    if (newNode) {
        // Crear nueva raíz
        SRNode* newRoot = new SRNode();
        newRoot->setIsLeaf(false);
        newRoot->setParent(nullptr);        newRoot->addChild(_root);
        newRoot->addChild(newNode);
        _root->setParent(newRoot);
        newNode->setParent(newRoot);
        newRoot->updateBoundingVolumes();
        _root = newRoot;
    }
}

bool SRTree::search(const Point& point) const {
    if (!_root) return false;
    return searchRecursive(_root, point);
}

bool SRTree::searchRecursive(SRNode* node, const Point& point) const {
    if (node->getIsLeaf()) {
        for (Point* p : node->getPoints()) {
            bool equal = true;
            for (std::size_t i = 0; i < DIM; ++i) {
                if (std::abs((*p)[i] - point[i]) > 1e-6f) {
                    equal = false;
                    break;
                }
            }
            if (equal) return true;
        }
        return false;
    } else {
        for (SRNode* child : node->getChildren()) {
            if (pointIntersectsMBB(point, child->getBoundingBox())) {
                if (searchRecursive(child, point)) return true;
            }
        }
        return false;
    }
}

bool SRTree::pointIntersectsMBB(const Point& point, const MBB& box) const {
    for (std::size_t i = 0; i < DIM; ++i) {
        if (point[i] < box.minCorner[i] - 1e-6f || point[i] > box.maxCorner[i] + 1e-6f) {
            return false;
        }
    }
    return true;
}

bool SRTree::pointIntersectsSphere(const Point& point, const Sphere& sphere) const {
    float dist = Point::distance(point, sphere.center);
    return dist <= sphere.radius + 1e-6f;
}

bool SRTree::mbbIntersectsMBB(const MBB& box1, const MBB& box2) const {
    for (std::size_t i = 0; i < DIM; ++i) {
        if (box1.maxCorner[i] < box2.minCorner[i] - 1e-6f || 
            box1.minCorner[i] > box2.maxCorner[i] + 1e-6f) {
            return false;
        }
    }
    return true;
}

bool SRTree::sphereIntersectsMBB(const Sphere& sphere, const MBB& box) const {
    float distSq = 0.0f;
    for (std::size_t i = 0; i < DIM; ++i) {
        float coord = sphere.center[i];
        if (coord < box.minCorner[i]) {
            float diff = box.minCorner[i] - coord;
            distSq += diff * diff;
        } else if (coord > box.maxCorner[i]) {
            float diff = coord - box.maxCorner[i];
            distSq += diff * diff;
        }
    }
    return std::sqrt(distSq) <= sphere.radius + 1e-6f;
}

std::vector<Point*> SRTree::rangeQuery(const MBB& box) const {
    std::vector<Point*> result;
    if (_root) {
        rangeQueryMBBRecursive(_root, box, result);
    }
    return result;
}

void SRTree::rangeQueryMBBRecursive(SRNode* node, const MBB& box, std::vector<Point*>& result) const {
    if (node->getIsLeaf()) {
        for (Point* p : node->getPoints()) {
            if (pointIntersectsMBB(*p, box)) {
                result.push_back(p);
            }
        }
    } else {
        for (SRNode* child : node->getChildren()) {
            if (mbbIntersectsMBB(child->getBoundingBox(), box)) {
                rangeQueryMBBRecursive(child, box, result);
            }
        }
    }
}

std::vector<Point*> SRTree::rangeQuery(const Sphere& sphere) const {
    std::vector<Point*> result;
    if (_root) {
        rangeQuerySphereRecursive(_root, sphere, result);
    }
    return result;
}

void SRTree::rangeQuerySphereRecursive(SRNode* node, const Sphere& sphere, std::vector<Point*>& result) const {
    if (node->getIsLeaf()) {
        for (Point* p : node->getPoints()) {
            if (pointIntersectsSphere(*p, sphere)) {
                result.push_back(p);
            }
        }
    } else {
        for (SRNode* child : node->getChildren()) {
            if (sphereIntersectsMBB(sphere, child->getBoundingBox())) {
                rangeQuerySphereRecursive(child, sphere, result);
            }
        }
    }
}

std::vector<Point*> SRTree::kNearestNeighbors(const Point& point, std::size_t k) const {
    std::vector<std::pair<float, Point*>> candidates;
    if (_root) {
        kNNRecursive(_root, point, candidates);
    }
    
    // Ordenar por distancia
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    // Tomar los k más cercanos
    std::vector<Point*> result;
    for (std::size_t i = 0; i < std::min(k, candidates.size()); ++i) {
        result.push_back(candidates[i].second);
    }
    return result;
}

void SRTree::kNNRecursive(SRNode* node, const Point& point, std::vector<std::pair<float, Point*>>& candidates) const {
    if (node->getIsLeaf()) {
        for (Point* p : node->getPoints()) {
            float dist = Point::distance(point, *p);
            candidates.emplace_back(dist, p);
        }
    } else {
        // Crear lista de hijos ordenados por distancia mínima
        std::vector<std::pair<float, SRNode*>> childDistances;
        for (SRNode* child : node->getChildren()) {
            float minDist = calculateMinDistToMBB(point, child->getBoundingBox());
            childDistances.emplace_back(minDist, child);
        }
        
        std::sort(childDistances.begin(), childDistances.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        
        for (const auto& pair : childDistances) {
            kNNRecursive(pair.second, point, candidates);
        }
    }
}

float SRTree::calculateMinDistToMBB(const Point& point, const MBB& box) const {
    float distSq = 0.0f;
    for (std::size_t i = 0; i < DIM; ++i) {
        float coord = point[i];
        if (coord < box.minCorner[i]) {
            float diff = box.minCorner[i] - coord;
            distSq += diff * diff;
        } else if (coord > box.maxCorner[i]) {
            float diff = coord - box.maxCorner[i];
            distSq += diff * diff;
        }
    }
    return std::sqrt(distSq);
}