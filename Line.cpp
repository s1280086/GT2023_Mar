#ifndef LINE
#define LINE

#include <iostream>
#include <limits>
#include <math.h>
#include <array>
#include <vector>
#include "Point.cpp"

class Line{
    public:
      Point begin, end, mid;
      int SurfaceID;
      bool throught = false;
      int PairNo = -1;
    
      //std::array <int, 2> plane = {-1, -1};

      Line(){
        this->throught = false;
      };
      /*
      line(const point& Begin, const point& End){
        this->begin = Begin;
        this->end = End;
        this->throught = false;
      };
      */
      Line(Point Begin, Point End){
        this->begin = Begin;
        this->end = End;
        this->throught = false;
        this->mid = (this->begin+this->end)/2;
      }

      Line(Point Begin, Point End, int SurfaceID){
        this->begin = Begin;
        this->end = End;
        this->SurfaceID = SurfaceID;
        this->mid = (this->begin+this->end)/2;
      }

      bool judgecross(const Line& cd){
        /*(ccw(a,b,c) * ccw(a,b,d) <= 0)
          &&
          (ccw(c,d,a) * ccw(c,d,b) <= 0)
        */
        int abc, abd, cda, cdb;
        abc = ccw(this->begin, this->end, cd.begin);
        abd = ccw(this->begin, this->end, cd.end);
        cda = ccw(cd.begin, cd.end, this->begin);
        cdb = ccw(cd.begin, cd.end, this->end);
        //std::cout<<"abc = "<<abc<<", abd = "<<abd<<", cda = "<<cda<<", cdb = "<<cdb<<std::endl;
        if((abc * abd <= 0) && (cda * cdb <= 0)){
            //std::cout<<"now crossing"<<std::endl;
            return true;}
        else return false;
      };

      int ccw(const Point& a, const Point& b, const Point& c){
        double EPS = std::numeric_limits<double>::epsilon();
        //std::cout<<EPS<<std::endl;
        if(cross(b - a, c - a) > EPS) return +1;
        if(cross(b - a, c - a) < -EPS) return -1;
        if(dot(b - a, c - a) < -EPS) return +2;
        if(norm(b - a) + EPS < norm(c - a)) return -2;
        return 0;
      };
      double cross(const Point& ba, const Point& ca){
        double ExP = (ba.x * ca.y) - (ca.x * ba.y);
        //std::cout<<""<<ExP<<", ";
        return ExP;
      };
      double dot(const Point& ba, const Point& ca){
        double InP = ba.x*ca.x + ba.y*ca.y;
        return InP;
      };
      double norm(const Point& ba){
        double Norm = sqrt( (pow(ba.x-0, 2) + pow(ba.y-0, 2)) );
        return Norm;
      };

      bool operator==(Line anotherline){
        if((this->begin == anotherline.begin)
        &&(this->end == anotherline.end))return true;
        else{return false;}
      };

      Line SwitchLine(const std::pair<Line,Line>& Pair, int &NexFloor){
        if(*(this) == Pair.first){
          NexFloor = Pair.second.SurfaceID;
          return Pair.second;}
        else if(*(this) == Pair.second){
          NexFloor = Pair.first.SurfaceID;
          return Pair.first;}
        else{std::cout<<"Pair Line Not found";
          return *this;
        }
      };
      
};

#endif