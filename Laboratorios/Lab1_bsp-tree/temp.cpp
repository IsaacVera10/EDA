#include "Point.h"
#include "Line.h"

using namespace std;

int main(){
    cout<<"-------------------- Puntos --------------------"<<endl;
    
    Point3D<> p1(1, 2, 3);
    Point3D<> p2(4, 5, 6);

    cout << "Point 1: " << p1 << endl;
    cout << "Point 2: " << p2 << endl;

    cout<<endl;

    cout<< "Distance between Point 1 and Point 2: " << p1.distance(p2) << endl;

    cout<<"------------------------------------------------"<<endl;

    cout<<endl;

    cout<<"-------------------- Line --------------------"<<endl;

    LineSegment<> lineSegment(p1, p2);
    cout<<"LineSegment: " << lineSegment << endl<<endl;
    cout << "Line Segment Length: " << lineSegment.length() << endl;
    cout << "Line Segment Points: " << lineSegment.getP1() << " to " << lineSegment.getP2() << endl;

    cout<<"-----------------------------------------------"<<endl;

    cout<<endl;

    cout<<"-------------------- Vectores --------------------"<<endl;

    Vector3D<> vector1(p1);
    Vector3D<> vector2(p2);
    cout<<"Vector 1: " << vector1 << endl;
    cout<<"Vector 2: " << vector2 << endl<<endl;

    cout<<"Dot Product: " << vector1.dot(vector2) << endl;
    cout<<"Cross Product: " << vector1.cross(vector2) << endl<<endl;
    cout<<"Magnitude of Vector 1: " << vector1.magnitude() << endl;
    cout<<"unit vector 1: " << vector1.unit() << endl;
    cout<<"unit vector 2: " << vector2.unit() << endl;

    cout<<"Angle between Vector 1 and Vector 2: " << vector1.angle(vector2) << " radians" << endl;
    cout<<"-----------------------------------------------"<<endl;





    return 0;
}
