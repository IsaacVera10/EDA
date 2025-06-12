#include<iostream>
#include "Point.h"
#include "MBB.h"
#include "Sphere.h"
using namespace std;

void prueba_MBB(){
    // Crear un punto y una caja MBB
    Point p1({2.0f, 3.0f});
    Point p2({6.0f, 7.0f});
    MBB box(p1, p2);
    cout << "Caja MBB inicial:\n";
    cout << "Min Corner: (" << box.minCorner[0] << ", " << box.minCorner[1] << ")\n";
    cout << "Max Corner: (" << box.maxCorner[0] << ", " << box.maxCorner[1] << ")\n";
    cout<<endl;
    // Expandir la caja para incluir otro punto
    Point p3({8.0f, 10.0f});
    box.expandToInclude(p3);
    cout << "Caja MBB expandida:\n";
    cout << "Min Corner: (" << box.minCorner[0] << ", " << box.minCorner[1] << ")\n";
    cout << "Max Corner: (" << box.maxCorner[0] << ", " << box.maxCorner[1] << ")\n";

    // Calcular la distancia máxima desde un punto al MBB
    Point center({5.0f, 5.0f});
    float maxDist = MBB::maxDist(center, box);
    cout<< "Distancia máxima desde el punto al MBB: " << maxDist << endl;
}

void prueba_Sphere(){
    Sphere sphere(Point({7.0f, 8.0f}), 5.0f);
    cout << "Esfera inicial:\n";
    cout << "Centro: (" << sphere.center[0] << ", " << sphere.center[1] << ")\n";
    cout << "Radio: " << sphere.radius << "\n";
    cout << endl;
    // Expandir la esfera para incluir otra esfera
    Sphere otherSphere(Point({12.0f, 2.0f}), 1.0f);
    cout << "Otra esfera:\n";
    cout << "Centro: (" << otherSphere.center[0] << ", " << otherSphere.center[1] << ")\n";
    cout << "Radio: " << otherSphere.radius << "\n";
    cout << endl;
    sphere.expandToInclude(otherSphere);
    cout << "Esfera expandida:\n";
    cout << "Centro: (" << sphere.center[0] << ", " << sphere.center[1] << ")\n";
    cout << "Radio: " << sphere.radius << "\n";
    cout<<endl;
    // Probar la función de expansión con un punto
    Point pointToInclude({13.0f, 6.0f});
    sphere.expandToInclude(pointToInclude);
    cout << "Esfera expandida:\n";
    cout << "Centro: (" << sphere.center[0] << ", " << sphere.center[1] << ")\n";
    cout << "Radio: " << sphere.radius << "\n";

}

int main() {

    return 0;
}