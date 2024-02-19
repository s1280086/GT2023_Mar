#ifndef TRANSFORM
#define TRANSFORM

#include<iostream>
#include<vector>
//use sin cos tan
#define _USE_MATH_DEFINES
#include <math.h>

#include"Line.cpp"
#include "RobotState.h"

using namespace std;

//this class will member of "plane object"
class trans{
    public:
      int planeno, xdeg=0, ydeg=0, zdeg=0;
      Point offset;

      //for 
      Point res, ress;
      RobotState afterpoint;
    /*
    ワールド座標系から各プレーンフレームへの変換
    逆をするためには符号の位置に注意
    x-axis transform
       1     0     0
       0    cosθ  sinθ  
       0   -sinθ  cosθ

    y-axis transform
     cosθ    0   -sinθ
       0     1     0  
     sinθ    0    cosθ

     z-axis transform
      cosθ  sinθ   0
     -sinθ  cosθ   0
       0     0     1
    */
      trans(){};
      trans(int xde, int yde, int zde, Point offs){
        this->xdeg = xde;//
        this->ydeg = yde;
        this->zdeg = zde;
        this->offset = offs;
      };

      Point ytrans(Point p){//local flame
        res.x = p.x*( cos(this->ydeg*(M_PI/180)) ) + p.z*(-sin(this->ydeg*(M_PI/180)) );
        res.y = p.y;
        res.z = p.x*( sin(this->ydeg*(M_PI/180)) ) + p.z*( cos(this->ydeg*(M_PI/180)) );
        return res;
      };

      Point yrevtrans(Point p){//local flame
        res.x = p.x*( cos(this->ydeg*(M_PI/180)) ) + p.z*( sin(this->ydeg*(M_PI/180)) );
        res.y = p.y;
        res.z = p.z*(-sin(this->ydeg*(M_PI/180)) ) + p.z*( cos(this->ydeg*(M_PI/180)) );
        return res;
      };

      Point ztrans(Point p){//local flame
        res.x = p.x*( cos(this->zdeg*(M_PI/180)) ) + p.y*( sin(this->zdeg*(M_PI/180)) );
        res.y = p.x*(-sin(this->zdeg*(M_PI/180)) ) + p.y*( cos(this->zdeg*(M_PI/180)) );
        res.z = p.z;
        return res;
      };

      Point zrevtrans(Point p){//local flame
        res.x = p.x*( cos(this->zdeg*(M_PI/180)) ) + p.y*(-sin(this->zdeg*(M_PI/180)) );
        res.y = p.x*( sin(this->zdeg*(M_PI/180)) ) + p.y*( cos(this->zdeg*(M_PI/180)) );
        res.z = p.z;
        return res;
      };

      

      Point Rotate(Point p){//input local and output world
        //p -= this->offset;
        ress = this->ytrans(p);
        //ress = this->xtrans(res);
        ress = this->ztrans(ress);
        ress += this->offset;
        return ress;
      };

      Point RevRotate(Point p){
        //p -= this->offset;
        ress = this->zrevtrans(p);
        //ress = this->xrevtrans(res);
        ress = this->yrevtrans(ress);
        ress += this->offset;
        return ress;
      }


/*
    //変換後のpoint 変換後のflame.FotF(変換前のflame, 変換前のpoint)
      RobotState FtoF(trans beforeflame, RobotState beforepoint, RobotParam p, line beforeline){
        trans *afterflame = this;
        beforepoint.centor -= beforeflame.offset;
        //変換後の展開図上(2D)でのローカル座標を求める
        //afterpoint.centor = (beforeflame.offset + beforepoint.centor) - this->offset;
        afterpoint.centor = (beforeflame.offset + beforepoint.centor) - beforeline.begin;
        
        //展開図(2D)から三次元ローカル座標系(2.5D)へと回転変換(y->x->zの順)
        afterflame->ydeg -= beforeflame.ydeg;
        if(afterflame->ydeg <= -180) afterflame->ydeg += 360;
        else if(afterflame->ydeg>180) afterflame->ydeg -=360;
        //afterpoint.centor = afterflame->Rotate(afterpoint.centor + this->offset);
        afterpoint.centor = afterflame->Rotate(afterpoint.centor + this->offset);
        afterpoint.centor += (beforeline.begin - this->offset);
        afterpoint.theta = beforepoint.theta;

        afterpoint.MomentEdge3D(p, *afterflame);
        afterflame->ydeg += beforeflame.ydeg;

        return afterpoint;
      };
*/
};

#endif

