#ifndef POINT
#define POINT

#include "math.h"

class Point{
    public:
      double x, y, z;
      const double epsxyz = 0.07071;
      Point(){
        this->x = 0; this->y = 0; this->z = 0;
      };
      Point(double x, double y){
        this->x = x; this->y = y; this->z = 0;
      }

      Point(double x, double y, double z){
        this->x = x; this->y = y; this->z = z;
      };

      double distance(Point point1){
        double dis = sqrt( pow(point1.x - x, 2) + pow(point1.y - y, 2) );
        return dis;
      };

      const Point operator+(const Point& cppoint)const{
        Point res;
        res.x = this->x + cppoint.x;
        res.y = this->y + cppoint.y;
        res.z = this->z + cppoint.z;
        return res;
      };
      const Point operator-(const Point& cppoint)const{
        Point res;
        res.x = this->x - cppoint.x;
        res.y = this->y - cppoint.y;
        res.z = this->z - cppoint.z;
        return res;
      };

      const Point operator/(const int& dev)const{
        Point res;
        res.x = this->x / dev;
        res.y = this->y / dev;
        res.z = this->z / dev;
        return res;
      }

      Point& operator+=(const Point& cppoint){
        this->x += cppoint.x;
        this->y += cppoint.y;
        this->z += cppoint.z;
        return *this;
      };
      Point& operator-=(const Point& cppoint){
        this->x -= cppoint.x;
        this->y -= cppoint.y;
        this->z -= cppoint.z;
        return *this;
      };
      Point& operator=(const Point& cppoint){
        if(this == &cppoint)return *this;
        this->x = cppoint.x;
        this->y = cppoint.y;
        this->z = cppoint.z;
        return *this;
      };

      bool operator==(const Point& anotherpoint){
        if((this->x == anotherpoint.x)
        &&(this->y == anotherpoint.y)
        &&(this->z == anotherpoint.z))return true;
        else{return false;}
      };
};

#endif