#ifndef OBSTACLE
#define OBSTACLE

#include <iostream>
#include <math.h>
#include <vector>

#include "RobotState.h"
#include "Guide.cpp"

#include "Point.cpp"
#include "Line.cpp"

#include "3dtransform.cpp"

using namespace std;

//#include <cmath>

class Obstacle{
    public:
      int planeno = -1;
      vector<Point> vertex;
      vector<Line> obsedges;
      trans rotation;
      Point afterpoint;
      Obstacle(){};
      Obstacle(RobotState p){};
      virtual void shape(){cout<<"obstacle";};
      virtual bool checkcoll(vector<Line> robotedges){return true;};
};

class rectangle : public Obstacle{
    public:
      double sizex, sizey;
      Point pa, pb, pc, pd, pp, afterpoint;
      Line lab, lbc, lcd, lda;
      vector<Line> obsedges;
      trans rotation;
      void shape()override{cout<<"rectangle";};
      rectangle(){};

      rectangle(double insizex, double insizey, int inplaneno){
        this->sizex = insizex;
        this->sizey = insizey;
        this->planeno = inplaneno;
        this->vertex.clear();
        this->pa = Point(0, 0);                     this->vertex.push_back(this->pa);
        this->pb = Point(0, this->sizey);           this->vertex.push_back(this->pb);
        this->pc = Point(this->sizex, this->sizey); this->vertex.push_back(this->pc);
        this->pd = Point(this->sizex, 0);           this->vertex.push_back(this->pd);
      };

      void setrotation(int xdeg, int ydeg, int zdeg, Point offset){

        trans hoge(xdeg,ydeg,zdeg, offset);
        this->rotation = hoge;

        this->vertex.clear();
        this->pa = hoge.Rotate(Point(0, 0, 0)) ;
          this->vertex.push_back(this->pa);
        this->pb = hoge.Rotate(Point(0, this->sizey, 0));
          this->vertex.push_back(this->pb);
        this->pc = hoge.Rotate(Point(this->sizex, this->sizey, 0));
          this->vertex.push_back(this->pc);
        this->pd = hoge.Rotate(Point(this->sizex, 0, 0));
          this->vertex.push_back(this->pd);
      };
      
      void PushEdgeVec(){
        this->obsedges.clear();
        this->lab.begin = this->pa; this->lab.end = this->pb;
        this->lbc.begin = this->pb; this->lbc.end = this->pc;
        this->lcd.begin = this->pc; this->lcd.end = this->pd;
        this->lda.begin = this->pd; this->lda.end = this->pa;

        this->obsedges.push_back(this->lab);
        this->obsedges.push_back(this->lbc);
        this->obsedges.push_back(this->lcd);
        this->obsedges.push_back(this->lda);
      }

      bool checkcoll(vector<Line> robotedges) override{
        //unthrought edge か throught edgeか確認
        for(auto itr : this->obsedges){
          if(itr.throught == false){
            for(auto itrr : robotedges){
              if(itr.judgecross(itrr) == true)return true;
            }
          }
        }
        return false;
      };

/**/
      void FrameTrans(RobotState& beforeframe, const Line& BeforeLine, const Line& AfterLine){
        beforeframe.center -= BeforeLine.begin;
        beforeframe.center += AfterLine.begin;
      };

};


#endif