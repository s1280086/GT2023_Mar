#ifndef PLOTRESULT
#define PLOTRESULT

#include <iostream>
#include <fstream>
#include <stack>
#include "Surface.cpp"


using namespace std;

#define Ob "./CurrentResult/PlaneObs.txt"
#define Li "./CurrentResult/TransLine.txt"
#define MN "./CurrentResult/MacroNode.txt"
#define MP "./CurrentResult/MacroPath.txt"
#define Pl "./CurrentResult/Path.txt"
#define Re "./CurrentResult/RobotEdge.txt"


void PlotResult(vector<Surface> inPlanWS, vector<Surface> inTerrainWS, vector<vector<RobotState>> inMids, vector<RobotState> inMacroPath, stack<RobotState> inMicroPath, RobotParam inRparam){
    vector<Surface> PlanWorkSpace = inPlanWS;
    vector<Surface> TerrainWorkSpace = inTerrainWS;
    vector<RobotState> MacroPath = inMacroPath;
    stack<RobotState> path = inMicroPath;
    vector<vector<RobotState>> Mids = inMids;
    RobotParam Rparam = inRparam;
    RobotState hoge;


    //plot data for obstacle
    ofstream PlotPlaneAndObject(Ob);
    for(auto itr: TerrainWorkSpace){
      for(auto itrr: itr.PlnObs){
        for(auto itrrr: itrr.vertex){
          PlotPlaneAndObject<<itrrr.x<<" "<<itrrr.y<<" "<<itrrr.z<<endl;
        }
        PlotPlaneAndObject<<itrr.vertex.front().x<<" "<<itrr.vertex.front().y<<" "<<itrr.vertex.front().z<<endl<<endl<<endl;
        
      }
    }
    PlotPlaneAndObject.close();

  //plot data for TransLine
  Point BEGIN, END;
    ofstream PlotTransLine(Li);

    for(auto itr: PlanWorkSpace){
      for(auto itrr: itr.TransLine){
        //Line1
        BEGIN = TerrainWorkSpace.at(itrr.Pair.first.SurfaceID).OffRot.Rotate(itrr.Pair.first.begin);
        END   = TerrainWorkSpace.at(itrr.Pair.first.SurfaceID).OffRot.Rotate(itrr.Pair.first.end  );
        PlotTransLine<<BEGIN.x<<" "<<BEGIN.y<<" "<<BEGIN.z<<endl;
        PlotTransLine<<END.x  <<" "<<END.y  <<" "<<END.z  <<endl<<endl<<endl;
        //Line2
        BEGIN = TerrainWorkSpace.at(itrr.Pair.second.SurfaceID).OffRot.Rotate(itrr.Pair.second.begin);
        END   = TerrainWorkSpace.at(itrr.Pair.second.SurfaceID).OffRot.Rotate(itrr.Pair.second.end  );
        PlotTransLine<<BEGIN.x<<" "<<BEGIN.y<<" "<<BEGIN.z<<endl;
        PlotTransLine<<END.x  <<" "<<END.y  <<" "<<END.z  <<endl<<endl<<endl;
      }
    }
    PlotTransLine.close();



  //plot data for Macro Node
    ofstream PlotMacroNode(MN);
    Point BuffPoint;
    for(int i=0;i<Mids.size();i++){
      for(int j=0;j<Mids.at(i).size();j++){

        for(int k=i+1;k<Mids.size();k++){
          for(int l=0;l<Mids.at(k).size();l++){
            if(Mids.at(i).at(j).SurfaceID == Mids.at(k).at(l).SurfaceID){
              BuffPoint = TerrainWorkSpace.at(Mids.at(i).at(j).SurfaceID).OffRot.Rotate(Mids.at(i).at(j).center);
              PlotMacroNode<<BuffPoint.x<<" ";
              PlotMacroNode<<BuffPoint.y<<" ";
              PlotMacroNode<<BuffPoint.z<<endl;

              BuffPoint = TerrainWorkSpace.at(Mids.at(k).at(l).SurfaceID).OffRot.Rotate(Mids.at(k).at(l).center);
              PlotMacroNode<<BuffPoint.x<<" ";
              PlotMacroNode<<BuffPoint.y<<" ";
              PlotMacroNode<<BuffPoint.z<<endl<<endl<<endl;
            }
          }
        }

      }
    }
    PlotMacroNode.close();
  //plot data for Macro Path
    ofstream PlotMacroPath(MP);
      for(auto itr : MacroPath){
        BuffPoint = TerrainWorkSpace.at(itr.SurfaceID).OffRot.Rotate(itr.center);
        PlotMacroPath<<BuffPoint.x<<" ";
        PlotMacroPath<<BuffPoint.y<<" ";
        PlotMacroPath<<BuffPoint.z<<endl;
      }
    PlotMacroPath.close();


  //plot data for path(center and edges)
    ofstream PlotResultPath(Pl);
    ofstream PlotResultEdge(Re);

    while(path.empty() != true){
        hoge.center = TerrainWorkSpace.at(path.top().SurfaceID).OffRot.Rotate(path.top().center);
        PlotResultPath<<*hoge.x<<" "<<*hoge.y<<" "<<*hoge.z<<endl;
        path.top().MomentEdge3D(Rparam, TerrainWorkSpace.at(path.top().SurfaceID).OffRot);
        for(int i=0;i<path.top().vertex.size();i++){
          PlotResultEdge<<path.top().vertex.at(i).x<<" "<<path.top().vertex.at(i).y<<" "<<path.top().vertex.at(i).z<<endl;
        }
        PlotResultEdge<<path.top().vertex.at(0).x<<" "<<path.top().vertex.at(0).y<<" "<<path.top().vertex.at(0).z<<endl<<endl<<endl;

        path.pop();
    }


    PlotResultPath.close();
    PlotResultEdge.close();
};
#endif