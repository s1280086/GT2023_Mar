#ifndef SURFACE
#define SURFACE

#include "TransLine.cpp"
#include "obstacle.cpp"
#include "3dtransform.cpp"

class Surface{
    public:
      int SurfaceID;
      //double SurfaceCost;
      trans OffRot;
      vector<rectangle> PlnObs;
      //vector<pair<Line,Line>> TransLine;
      vector<TransLine> TransLine;
      
    Surface(rectangle Surfacein){
        PlnObs.push_back(Surfacein);
        this->SurfaceID = Surfacein.planeno;
        this->OffRot = Surfacein.rotation;
    };

    void FrameTrans(RobotState& beforeframe, const Line& BeforeLine, const Line& AfterLine){
        beforeframe.center -= BeforeLine.begin;
        beforeframe.center += AfterLine.begin;
    };


};
#endif