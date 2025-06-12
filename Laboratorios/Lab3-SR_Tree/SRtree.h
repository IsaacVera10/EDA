#ifndef SRTREE_H
#define SRTREE_H

#include <vector>
#include <limits>
#include <algorithm>
#include <string>
#include <cmath>
#include <stdexcept>
#include "Point.h"
#include "MBB.h"
#include "Sphere.h"
#include <queue>


using pares = std::pair<float, Point*>;
using max_heap = std::priority_queue<pares, std::vector<pares>, std::function<bool(pares, pares)>>;


class SRNode {
private:
    MBB       _boundingBox;
    Sphere _boundingSphere;
    SRNode*        _parent;
    std::vector<Point*>    _points;
    std::vector<SRNode*> _children;
    bool _isLeaf;

public:
    SRNode(SRNode* padre = nullptr, bool hoja = true): 
    _boundingBox(), _boundingSphere(), _parent(padre), _points(), _children(), _isLeaf(hoja) {}

    bool    getIsLeaf() const { return _isLeaf; }
    SRNode* getParent() const { return _parent; }

    const    MBB& getBoundingBox   () const { return    _boundingBox; }
    const Sphere& getBoundingSphere() const { return _boundingSphere; }
    const std::vector< Point*>&   getPoints() const { return   _points; }
    const std::vector<SRNode*>& getChildren() const { return _children; }
    std::size_t getNumPoints  () const { return   _points.size(); }
    std::size_t getNumChildren() const { return _children.size(); }

    void setBoundingBox   (const    MBB&    box) { _boundingBox    =    box; }
    void setBoundingSphere(const Sphere& sphere) { _boundingSphere = sphere; }
    void setParent        (      SRNode* parent) {         _parent = parent; }
    void setIsLeaf        (bool          isLeaf) {         _isLeaf = isLeaf; }

    // Insert algorithm
    SRNode* insert(Point& _data, std::size_t maxEntries);

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
};


SRNode* SRNode::insert(Point& dato, std::size_t maxEntries) {
    if (_isLeaf) { //Si el nodo actual es una hoja, entonces insertamos el punto directamente
        _points.push_back(&dato);

        // Actualizamos los volúmenes de contención
        if (_points.size() == 1) {//Si solo hay el punto que se acaba de insertar
            _boundingBox = MBB(dato);
            _boundingSphere = Sphere(dato, 0.0f);
        } else {//Hay más de un punto
            _boundingBox.expandToInclude(dato);
            _boundingSphere.expandToInclude(dato);
        }

        return this; //Retornamos este punto para saber dónde se insertó el dato
        /*Futura mejora para evaluar split*/

    } else {//Es un nodo interno
        if (_children.empty()) {
            SRNode* nuevo = new SRNode(this, true);
            _children.push_back(nuevo);
        }
        // Simplificamos la heurística de inserción: insertamos en el primer hijo
        SRNode* hijo = _children.front()->insert(dato, 0);//Retorna el nodo hijo donde se insertó el punto
        
        // Update de los volúmenes de contención
        _boundingBox.expandToInclude(hijo->getBoundingBox());
        _boundingSphere.expandToInclude(hijo->getBoundingSphere());
        return hijo;
    }
}

void SRTree::insert(const Point& point) {
    Point* nuevo = new Point(point);
    if (!_root) {//Si el árbol está vacío, creamos la raíz
        _root = new SRNode(nullptr, true);//También es una hoja
    }
    _root->insert(*nuevo, _maxEntries);
}

bool search_recursive(const SRNode* node, const Point& point) {
    if (node->getIsLeaf()) {//Nodo hoja
        // Verificamos si el punto está en la lista de puntos del nodo
        for (Point* punt : node->getPoints()) {
            bool find = true;
            for (size_t i = 0; i < DIM; ++i) {
                if (std::abs((*punt)[i] - point[i]) > EPSILON) {
                    find = false;
                    break;
                }
            }
            if (find) return true;
        }
        return false;
    } else {
        for (SRNode* child : node->getChildren()) if (search_recursive(child, point)) return true;
        return false;
    }
}

bool SRTree::search(const Point& point) const {
    if (!_root) return false;
    return search_recursive(_root, point);
}

void range_box_recursive(const SRNode* node, const MBB& box_query, std::vector<Point*>& output) {
    if (!node->getBoundingBox().intersecta(box_query)) return;

    // Si es una hoja, revisamos los puntos data
    if (node->getIsLeaf()) {
        for (Point* i : node->getPoints()) if (box_query.contiene(*i)) output.push_back(i);
        return;
    } else for (const SRNode* child : node->getChildren()) range_box_recursive(child, box_query, output);

}

std::vector<Point*> SRTree::rangeQuery(const MBB& box) const {
    std::vector<Point*> result;
    if (!_root) return result; // Si el árbol está vacío, retornamos un vector vacío

    range_box_recursive(_root, box, result);
    return result;
}

void range_esphere_recursive(const SRNode* node, const Sphere& sphere_query, std::vector<Point*>& output) {
    const MBB& node_box = node->getBoundingBox();
    const Sphere& node_sphere = node->getBoundingSphere();

    float dist_c_c = Point::distance(sphere_query.center, node_sphere.center);
    if(dist_c_c > (sphere_query.radius + node_sphere.radius)) return; 

    if(node->getIsLeaf()) { // Si es una hoja, revisamos los puntos data
        for (Point* p : node->getPoints()) if (Point::distance(sphere_query.center, *p) <= sphere_query.radius + EPSILON) output.push_back(p);
    } else { // Si no es una hoja, recorremos los hijos
        for (const SRNode* child : node->getChildren()) range_esphere_recursive(child, sphere_query, output);
    }
}

std::vector<Point*> SRTree::rangeQuery(const Sphere& sphere) const {
    std::vector<Point*> result;
    if (!_root) return result; // Si el árbol está vacío, retornamos un vector vacío

    range_esphere_recursive(_root, sphere, result);
    return result;
}

std::vector <Point*> SRTree::kNearestNeighbors(const Point& query, std::size_t k) const {
    std::vector<Point*> resultado;
    if (!_root || k == 0) return resultado;

    std::vector<std::pair<float, Point*>> distancias;

    for (Point* p : _root->getPoints()) {// Recorremos los puntos del nodo raíz
        float d = Point::distance(query, *p);// Calculamos la distancia al punto
        distancias.emplace_back(d, p); // Almacenamos la distancia y el puntero al punto
    }
    std::sort(distancias.begin(), distancias.end(), [](auto& a, auto& b){ return a.first < b.first; }); //Ordenamos las distancias de menor a mayor

    if (k > distancias.size()) k = distancias.size();

    for (std::size_t i = 0; i < k; ++i) resultado.push_back(distancias[i].second);// Agregamos los k puntos más cercanos al resultado
    return resultado;
}
#endif // SRTREE_H