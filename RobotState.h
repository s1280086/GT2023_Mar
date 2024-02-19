//RobotState.h

#ifndef ROBOTSTATE
#define ROBOTSTATE

#include <vector>
#include <queue>
#include "RobotParam.h"
#include "Line.cpp"

//#include "Guide.cpp"
//#include "3dtransform.cpp"

//相互インクルード
//#include "Guidere.h"
class Guide;
class trans;

//for sin() cos() tan()
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

class RobotState{
    public:
        Point center;
        double *x, *y, *z;
        double theta, score, epsxy = 0.01, epstheta = 15;
        int flg=-1, BuffNodeNo=-1, step, SurfaceID, NodeNo;
        

        
        Point fr, fl, rr, rl;
        Line Front, Rear, Left, Right;
        vector<Line> edges;
        vector<Point> vertex;

        RobotState();
        explicit RobotState(double inx, double iny, double inz, int intheta, int insurfaceid);
        //copy constructer
        RobotState(const RobotState& cprobotstate);
        RobotState& operator=(const RobotState& cprobotstate);
        bool operator==(const RobotState& cprobotstate);

        RobotState& update(RobotState& nex, RobotParam& p, double omega);
        bool checkreach(RobotState& another);
        bool checkalready(vector<Guide>& alreadylist);
        int checkalready(priority_queue<Guide, vector<Guide>, greater<Guide>>& alreadylist);
        void MomentEdge2D(RobotParam& p);
        void MomentEdge3D(RobotParam& p, trans& offsetrotate);

        ~RobotState();
};

#endif