#include <array>
#include <iostream>

constexpr std::size_t DIM = 768;

struct Point {
    std::array<float, DIM> coords;
    Point() { coords.fill(0.0f); }
    Point(const std::array<float, DIM>& c) : coords(c) {}
};

struct MBB {
    Point minCorner, maxCorner;
    MBB() = default;
    MBB(Point min, Point max) : minCorner(min), maxCorner(max) {}
};

int main() {
    std::array<float, DIM> cmin, cmax;
    cmin.fill(1.0f);
    cmax.fill(2.0f);
    
    // This should cause most vexing parse
    MBB box(Point(cmin), Point(cmax));
    
    std::cout << "Success!" << std::endl;
    return 0;
}
