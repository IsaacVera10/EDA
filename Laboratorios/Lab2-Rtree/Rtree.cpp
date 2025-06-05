#include "Rtree.h"
using namespace std;

//---------------------- MBB --------------------

float MBB::area() const {
    return (upper.x - lower.x) * (upper.y - lower.y);
}

float MBB::semiPerimeter() const {
    return (upper.x - lower.x) + (upper.y - lower.y);
}

float MBB::distanceTo(const Point &p) const {
    float dx = max({ lower.x - p.x, 0.0f, p.x - upper.x });
    float dy = max({ lower.y - p.y, 0.0f, p.y - upper.y });
    return sqrt(dx * dx + dy * dy);
}

float MBB::deltaSemiPerimeter(const Point &p) const {
    MBB temp = *this;
    temp.expandToInclude(p);
    return temp.semiPerimeter() - semiPerimeter();
}

void MBB::expandToInclude(const Point &p) {
    lower.x = min(lower.x, p.x);
    lower.y = min(lower.y, p.y);
    upper.x = max(upper.x, p.x);
    upper.y = max(upper.y, p.y);
}

void MBB::expandToInclude(const MBB &other) {
    lower.x = min(lower.x, other.lower.x);
    lower.y = min(lower.y, other.lower.y);
    upper.x = max(upper.x, other.upper.x);
    upper.y = max(upper.y, other.upper.y);
}

float MBB::intersects(const MBB &other) const {
    float l_x = max(0.0f, min(upper.x, other.upper.x) - max(lower.x, other.lower.x));
    float l_y = max(0.0f, min(upper.y, other.upper.y) - max(lower.y, other.lower.y));
    return l_x * l_y;
}

MBB MBB::computeFromPoints(const vector<Point> &pts) {
    assert(!pts.empty());
    Point low = pts[0];
    Point up = pts[0];
    for (const auto &pt : pts) {
        low.x = min(low.x, pt.x);
        low.y = min(low.y, pt.y);
        up.x = max(up.x, pt.x);
        up.y = max(up.y, pt.y);
    }
    return MBB(low, up);
}

MBB MBB::computeFromNodes(const vector<RNode*> &nodes) {
    assert(!nodes.empty());
    MBB result_mbr = nodes[0]->mbr;
    for (int i = 1; i < nodes.size(); ++i) {
        result_mbr.expandToInclude(nodes[i]->mbr);
    }
    return result_mbr;
}

MBB MBB::unionOf(const MBB &a, const MBB &b) {
    MBB result = a;
    result.expandToInclude(b);
    return result;
}


//---------------------- RNode --------------------
RNode* RNode::insert(const Point &p, uchar maxEntries) {
    if (isLeaf) {
        points.push_back(p);
        mbr.expandToInclude(p);

        if (points.size() > maxEntries) {
            return linearSplitLeaf(maxEntries);
        }
        return nullptr;
    } else {
        float MAX = numeric_limits<float>::max();
        RNode* bestChild = nullptr;
        for (RNode* child : children) {
            float delta = child->mbr.deltaSemiPerimeter(p);
            if (delta < MAX) {
                MAX = delta;
                bestChild = child;
            }
        }

        RNode* split = bestChild->insert(p, maxEntries);
        mbr.expandToInclude(p);

        if (split) {
            children.push_back(split);
            mbr.expandToInclude(split->mbr);
            if (children.size() > maxEntries) {
                return quadraticSplitInternal(maxEntries);
            }
        }
        return nullptr;
    }
}

vector<Point> RNode::search(const MBB &query) const {
    vector<Point> result;

    if (mbr.intersects(query) < 0.0f) return result; //Se admite roces

    if (isLeaf) {
        for (const auto &p : points) {
            if (p.x>= query.lower.x && p.x<= query.upper.x &&
                p.y>= query.lower.y && p.y <= query.upper.y) {
                result.push_back(p);
            }
        }
    } else {
        for (RNode* child : children) {
            vector<Point> childResult = child->search(query);
            result.insert(result.end(), childResult.begin(), childResult.end());
        }
    }
    return result;
}

RNode* RNode::linearSplitLeaf(uchar maxEntries) {
    int var1 = 0, var2 = 0;
    float max_dist = -1;


    for (size_t i = 0; i < points.size(); i++) {
        for (size_t j = i + 1; j < points.size(); j++) {
            float d = points[i].distanceTo(points[j]);
            if (d > max_dist) {
                max_dist = d;
                var1 = i;
                var2 = j;
            }
        }
    }

    RNode* newNode = new RNode(true);
    Point a = points[var1];
    Point b = points[var2];
    vector<Point> temp = points;

    vector<Point> groupA, groupB;
    for (const Point &pt : temp) {
        float da = pt.distanceTo(a);
        float db = pt.distanceTo(b);
        if ((groupA.size() < (maxEntries + 1) / 2 && da < db) || groupB.size() >= (maxEntries + 1) / 2)
            groupA.push_back(pt);
        else
            groupB.push_back(pt);
    }

    points = groupA;
    mbr = MBB::computeFromPoints(points);
    newNode->points = groupB;
    newNode->mbr = MBB::computeFromPoints(groupB);
    return newNode;
}

RNode* RNode::quadraticSplitInternal(uchar maxEntries) {
    size_t var1 = 0, var2 = 0;
    float max_perdida = -1;
    for (size_t i = 0; i < children.size(); i++) {
        for (size_t j = i + 1; j < children.size(); j++) {
            MBB unionMBB = MBB::unionOf(children[i]->mbr, children[j]->mbr);
            float perdida = unionMBB.area() - children[i]->mbr.area() - children[j]->mbr.area();
            if (perdida > max_perdida) {
                max_perdida = perdida;
                var1 = i;
                var2 = j;
            }
        }
    }

    RNode* newNode = new RNode(false);
    RNode* childA = children[var1];
    RNode* childB = children[var2];

    vector<RNode*> temp = children;
    children.clear();
    newNode->children.clear();

    children.push_back(childA);
    newNode->children.push_back(childB);

    for (size_t i = 0; i < temp.size(); ++i) {
        if (i == var1 || i == var2) continue;
        RNode* c = temp[i];

        float d1 = MBB::unionOf(MBB::computeFromNodes(children), c->mbr).area();
        float d2 = MBB::unionOf(MBB::computeFromNodes(newNode->children), c->mbr).area();

        if ((children.size() < (maxEntries + 1) / 2 && d1 < d2) || newNode->children.size() >= (maxEntries + 1) / 2)
            children.push_back(c);
        else
            newNode->children.push_back(c);
    }

    mbr = MBB::computeFromNodes(children);
    newNode->mbr = MBB::computeFromNodes(newNode->children);
    return newNode;
}

// --------------------- RTree --------------------

void RTree::insert(const Point &p) {
    RNode* split = root->insert(p, maxEntries);
    if (split) {
        RNode* newRoot = new RNode(false);
        newRoot->children.push_back(root);
        newRoot->children.push_back(split);
        newRoot->mbr = MBB::computeFromNodes(newRoot->children);
        root = newRoot;
    }
}

vector<Point> RTree::search(const MBB &query) const {
    return root->search(query);
}

vector<Point> RTree::kNN(const Point &query, uchar k) const {
    priority_queue<QueueEntry, vector<QueueEntry>, QueueEntryComparator> pq;
    vector<Point> result;
    pq.push({ root->mbr.distanceTo(query), true, root, Point() });

    while (!pq.empty() && result.size() < k) {
        QueueEntry current = pq.top();
        pq.pop();

        if (current.isNode) {
            RNode* node = current.node;
            if (node->isLeaf) {
                for (const Point &pt : node->points) {
                    float dist = pt.distanceTo(query);
                    pq.push({ dist, false, nullptr, pt });
                }
            } else {
                for (RNode* child : node->children) {
                    float dist = child->mbr.distanceTo(query);
                    pq.push({ dist, true, child, Point() });
                }
            }
        } else {
            result.push_back(current.pt);
        }
    }
    return result;
}