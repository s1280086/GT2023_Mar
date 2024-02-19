#include "Line.cpp"
#include "TransLine.cpp"
#include "RobotParam.h"
#include "RobotState.h"
#include "Obstacle.cpp"
#include "Surface.cpp"
#include "Astar.cpp"

#include <iostream>
#include <fstream>
#include <string>
#include <stack>

#include <math.h>
#define _USE_MATH_DEFINES

#define Ob "./CurrentResult/PlaneObs.txt"
#define Li "./CurrentResult/TransLine.txt"
#define MN "./CurrentResult/MacroNode.txt"
#define MP "./CurrentResult/MacroPath.txt"
#define Pl "./CurrentResult/Path.txt"
#define Re "./CurrentResult/RobotEdge.txt"

using namespace std;

//g++ main.cpp point.cpp line.cpp Guide.cpp RobotState.cpp obstacle.cpp 3dtransform.cpp -o out



int main(){

    bool goalflg = false;
    RobotState start, goal, hoge;


  //set robot param
    RobotParam Rparam;
    Rparam.t = 1.0;
    Rparam.v = 0.1;
    Rparam.omega = 30;
    Rparam.l = 0.40;
    Rparam.w = 0.50;
    Rparam.h = 0.15;
  //close set robot param



  vector<Surface> PlanWorkSpace, TerrainWorkSpace;
  Point off, deg;
  int SurfaceID;
  double xsize, ysize, inx,iny,inz;
  bool lab=0, lbc=0, lcd=0, lda=0;

  //Prepare Plane and Obs
  ifstream finPlane("./CurrentWorkSpace/WorkPlane.txt");
  ifstream finObs("./CurrentWorkSpace/WorkObs.txt");
  if(!finPlane)cout<<"ERROR Can't Open WorkPlane.txt"<<endl;
  if(!finObs)cout<<"ERROR Can't Open WorkObs.txt"<<endl;
  //make Plane
  while(!finPlane.eof()){
    finPlane >> SurfaceID>> xsize>>ysize>> off.x>>off.y>>off.z>> deg.x>>deg.y>>deg.z>> lab>>lbc>>lcd>>lda;
    //common process for 2D 3D
    rectangle buff(xsize, ysize, SurfaceID);
      buff.lab.throught = lab; buff.lbc.throught = lbc;
      buff.lcd.throught = lcd; buff.lda.throught = lda;
    //for PlanWorkSpace(2Dmap)
    buff.PushEdgeVec();
    Surface Buff(buff);
    PlanWorkSpace.push_back(Buff);

    //for TerrainWorkSpace(3Dmap)
    buff.setrotation(deg.x, deg.y, deg.z, off);
    buff.PushEdgeVec();
    Surface Buff2(buff);
    TerrainWorkSpace.push_back(Buff2);
  }
  finPlane.close();
  cout<<"End of make Plane"<<endl;

  cout<<"PlanWorkSpace.size"<<PlanWorkSpace.size()<<" ... TerrainWorkSpace.size"<<TerrainWorkSpace.size()<<endl;
  for(int i=0;i<PlanWorkSpace.size();i++){
    if(PlanWorkSpace.at(i).SurfaceID != i)cout<<"error";
  }

  //make Obs
  while(!finObs.eof()){
    finObs >> SurfaceID>> xsize>>ysize>> off.x>>off.y>>off.z>> deg.x>>deg.y>>deg.z>> lab>>lbc>>lcd>>lda;
    //common process
    rectangle buff(xsize, ysize, SurfaceID);
      buff.lab.throught = lab; buff.lbc.throught = lbc;
      buff.lcd.throught = lcd; buff.lda.throught = lda;
    //for PlanWorkSpace(2Dmap)
    buff.setrotation(0, 0, 0, off);
    buff.PushEdgeVec();

    PlanWorkSpace.at(SurfaceID).PlnObs.push_back(buff);

    //for TerrainWorkSpace(3Dmap)
    buff.setrotation(deg.x, deg.y, deg.z, off+TerrainWorkSpace.at(SurfaceID).OffRot.offset);
    buff.PushEdgeVec();

    TerrainWorkSpace.at(SurfaceID).PlnObs.push_back(buff);
    
  }
  finObs.close();
  cout<<"End of Make Obs"<<endl;
  //debug
  /*
  for(int i=0;i<PlanWorkSpace.size();i++){
    cout<<PlanWorkSpace.at(i).PlnObs.size()<<" ... ";
  }cout<<endl;
  for(int i=0;i<TerrainWorkSpace.size();i++){
    cout<<TerrainWorkSpace.at(i).PlnObs.size()<<" ... ";
  }cout<<endl;
  */
  

  //make TransitionLines
  vector<TransLine> TransLines;
  
  ifstream finTransLine("./CurrentWorkSpace/WorkTransLine.txt");
    if(!finTransLine)cout<<"ERROR WorkTransLine.txt Can't Open"<<endl;
    vector<vector<RobotState>> Mids;
    vector<RobotState> Mid;
    RobotState formacro;
    Point mid;
    int cnt = 0;
    while(!finTransLine.eof()){
      //Line1
        finTransLine >>SurfaceID>>inx>>iny>>inz;
        Point BuffBegin1(inx, iny, inz);
        finTransLine >>inx>>iny>>inz;
        Point BuffEnd1(inx, iny, inz);
        Line BuffLine1(BuffBegin1, BuffEnd1, SurfaceID);
      //Line2
        finTransLine >>SurfaceID>>inx>>iny>>inz;
        Point BuffBegin2(inx, iny, inz);
        finTransLine >>inx>>iny>>inz;
        Point BuffEnd2(inx, iny, inz);
        Line BuffLine2(BuffBegin2, BuffEnd2, SurfaceID);
      //make and push PairLine
      TransLine PairLine(BuffLine1, BuffLine2, cnt++);
      TransLines.push_back(PairLine);


      formacro.center = BuffLine1.mid;
      formacro.SurfaceID = BuffLine1.SurfaceID;
      Mid.push_back(formacro);

      formacro.center = BuffLine2.mid;
      formacro.SurfaceID = BuffLine2.SurfaceID;
      Mid.push_back(formacro);

      Mids.push_back(Mid);
      Mid.clear();
      }
  finTransLine.close();
  cout<<"End of Make TransLines and ConPlaneMap"<<endl;
  for(auto TL : TransLines){
    PlanWorkSpace.at(TL.Pair.first.SurfaceID).TransLine.push_back(TL);
    //cout<<TL.Pair.first.SurfaceID<<", "<<TL.Pair.second.SurfaceID<<"___";
    swap(TL.Pair.first, TL.Pair.second);
    //cout<<TL.Pair.first.SurfaceID<<", "<<TL.Pair.second.SurfaceID<<endl;
    PlanWorkSpace.at(TL.Pair.first.SurfaceID).TransLine.push_back(TL);
    cout<<TL.PairNo<<endl;
  }

  //debug
  /*
  for(int i=0;i<PlanWorkSpace.size();i++){
    for(int j=0;j<PlanWorkSpace.size();j++){
      if(ConSurfaceTable.at(i).at(j) == true)cout<<"1 ";
      else if(ConSurfaceTable.at(i).at(j) ==false)cout<<"0 ";
      else{cout<<"ERROR";}
    }cout<<endl;
  }
  */
  //debug
  /*
  for(auto itr: PlanWorkSpace){
    for(auto itrr: itr.TransLine){
      cout<<itrr.Pair.first.SurfaceID<<", "<<itrr.Pair.second.SurfaceID<<endl;
    }
    cout<<endl;
  }*/

  //set start and goal
    cout<<"please input local-flame start x(double), y(double), theta(double), SurfaceID(int 0=1f, 1=slope, 2=2f)\n";
    cin >> *start.x >> *start.y >> start.theta >> start.SurfaceID;
    start.step=0;
    cout<<"local-flame start x = "<<start.center.x<<", y = "<<start.center.y<<", z = "<<start.center.z<<", SurfaceID = "<<start.SurfaceID<<endl;
    cout<<endl;
    
    cout<<"please input local-flame goal  x(double), y(double), theta(double), SurfaceID(int 0=1f, 1=slope, 2=2f)\n";
    cin >> *goal.x  >> *goal.y  >> goal.theta  >> goal.SurfaceID;
    cout<<"local-flame  goal x = "<< goal.center.x<<", y = "<< goal.center.y<<", z = "<< goal.center.z<<", SurfaceID = "<<goal.SurfaceID<<endl;
    cout<<endl;

    Mid.push_back(start);
    Mids.push_back(Mid);
    Mid.clear();
    Mid.push_back(goal);
    Mids.push_back(Mid);

    //debug
    int i=0;
    for(auto itr : Mids){
      cout<<Mids.at(i++).size()<<", ";
    }cout<<endl;
    i=0;

    //make table
    vector<vector<double>> PrePlanTable(Mids.size(), vector<double>(Mids.size(), INFINITY));
    /*        case of start = 1F, goal = 3F
                0,3     3,2     0,4     4,1     1,5     5,2      0       2
             | Pair1 | Pair2 | Pair3 | Pair4 | Pair5 | Pair6 | Start | Goal  |
     | Pair1 |   x   |   o   |   o   | null  | null  | null  |   o   | null  |
     | Pair2 |       |   x   | null  | null  | null  |   o   | null  |   o   |
     | Pair3 |       |       |   x   |   o   | null  | null  |   o   | null  |
     | Pair4 |       |       |       |   x   |   o   | null  | null  | null  |
     | Pair5 |       |       |       |       |   x   |   o   | null  | null  |
     | Pair6 |       |       |       |       |       |   x   | null  |   o   |
     | Start |       |       |       |       |       |       |   x   | null  |
     | Goal  |       |       |       |       |       |       |       |   x   |
    */

    for(i=0;i<Mids.size();i++){
      for(int j=0;j<Mids.at(i).size();j++){
        //cout<<Mids.at(i).at(j).center.x<<", "<<Mids.at(i).at(j).center.y<<endl;
        for(int k=i+1;k<Mids.size();k++){
          for(int l=0;l<Mids.at(k).size();l++){
            if(Mids.at(i).at(j).SurfaceID == Mids.at(k).at(l).SurfaceID){
              //if(Mids.at(k).at(l).SurfaceID == 2){cout<<"hoge";}
              PrePlanTable.at(i).at(k) = Mids.at(i).at(j).center.distance(Mids.at(k).at(l).center);
              
              //斜面のコストを傾斜によって定数倍する
              
              /*
              //スロープの長さ依存
              if(TerrainWorkSpace.at(Mids.at(i).at(j).SurfaceID).OffRot.ydeg != 0){
                PrePlanTable.at(i).at(k) *= (1+(0.5*TerrainWorkSpace.at(Mids.at(i).at(j).SurfaceID).OffRot.ydeg));
              }
              */
/*
              //スロープの傾斜依存(角度によって指数関数的に大きくなる)
              if(TerrainWorkSpace.at(Mids.at(i).at(j).SurfaceID).OffRot.ydeg != 0){
                PrePlanTable.at(i).at(k) *= pow(2.0, (TerrainWorkSpace.at(Mids.at(i).at(j).SurfaceID).OffRot.ydeg) );
              }
*/
              PrePlanTable.at(k).at(i) = PrePlanTable.at(i).at(k);
            }
            cnt++;
          }
        }

      }
    }

    //debug
    cout<<"Macro Planning Table"<<endl;
    for(i=0;i<PrePlanTable.size();i++){
      cout<<i<<"__";
      for(int j=0;j<PrePlanTable.size();j++){
        if(PrePlanTable.at(i).at(j) == INFINITY)cout<<"-1, ";
        else{cout<<PrePlanTable.at(i).at(j)<<", ";}
      }cout<<endl;
    }

    class Node{
      public:
        int number, parent;
        double cost;
        Node(){}
        Node(int innumber, int inpar, double incost){
          this->number = innumber;
          this->parent = inpar;
          this->cost = incost;
        }
        Node& operator = (const Node &cpNode){
          this->number = cpNode.number;
          this->parent = cpNode.parent;
          this->cost = cpNode.cost;
          return *this;
        }
        bool operator > (const Node &cpNode)const{
          if(cost > cpNode.cost)return true;
          else{return false;}
        }
    };
    priority_queue<Node, vector<Node>, greater<Node>> OpenList;
    Node Start(PrePlanTable.size()-2, -2, 0), BuffNode(-1, -1, 0),NexNode;

    vector<Node> ClosedList(PrePlanTable.size(), BuffNode);
    
    ClosedList.at(Start.number) = Start;
    OpenList.push(Start);
    //execute dijkstra
    while(OpenList.empty() != true){
      BuffNode = OpenList.top();
      OpenList.pop();

      for(i=0;i<PrePlanTable.size();i++){
        if(PrePlanTable.at(BuffNode.number).at(i) != INFINITY){
          //cout<<BuffNode.number<<" to "<<i<<"  ";
          //cout<<PrePlanTable.at(BuffNode.number).at(i)<<"___";
          NexNode = BuffNode;
          NexNode.parent = BuffNode.number;
          NexNode.number = i;
          NexNode.cost += PrePlanTable.at(BuffNode.number).at(i);
          
          if((ClosedList.at(NexNode.number).parent == -1) || (ClosedList.at(NexNode.number).cost > NexNode.cost)){
            //cout<<"Update ClosedList  ";
            ClosedList.at(NexNode.number) = NexNode;
            OpenList.push(NexNode);
          }
        }//cout<<endl;
      }

    }
    //debug
    cout<<endl<<"Result ClosedList"<<endl;
    for(auto itr: ClosedList){
      cout<<itr.number<<"  ";
      cout<<itr.parent<<"  ";
      cout<<itr.cost<<endl;
    }cout<<endl;

    //reconstruct Macro Path
    int preSurfaceID = goal.SurfaceID, nexSurfaceID;
    nexSurfaceID = preSurfaceID;
    RobotState BuffRS;
    vector<RobotState> MacroPath;
    MacroPath.push_back(goal);

    NexNode = ClosedList.back();//goal
    while(BuffNode.parent != -2){//find start
      BuffNode = NexNode;
      preSurfaceID = nexSurfaceID;
      for(i=0;i<PlanWorkSpace.size();i++){
        for(int j=0;j<PlanWorkSpace.at(i).TransLine.size();j++){
          if((PlanWorkSpace.at(i).TransLine.at(j).PairNo == BuffNode.parent) && (PlanWorkSpace.at(i).TransLine.at(j).Pair.second.SurfaceID == preSurfaceID)){
            int hogei= 1;
            PlanWorkSpace.at(i).TransLine.at(j).SetPriority(hogei);
            PlanWorkSpace.at(i).TransLine.at(j).heuristic = ClosedList.back().cost - ClosedList.at(BuffNode.parent).cost;
            NexNode = ClosedList.at(BuffNode.parent);
            nexSurfaceID = PlanWorkSpace.at(i).TransLine.at(j).Pair.first.SurfaceID;
            BuffRS.center = PlanWorkSpace.at(i).TransLine.at(j).Pair.first.mid;
            BuffRS.SurfaceID = PlanWorkSpace.at(i).TransLine.at(j).Pair.first.SurfaceID;
            MacroPath.push_back(BuffRS);
            
            //debug
            //cout<<"set priority, ";
            //cout<<", "<<PlanWorkSpace.at(i).TransLine.at(j).PairNo;
            //cout<<", "<<preSurfaceID<<", "<<nexSurfaceID<<endl;
            break;
          }
          if(preSurfaceID == start.SurfaceID){
            NexNode = ClosedList.at(ClosedList.size()-2);
            nexSurfaceID = start.SurfaceID;
            break;
          }
        }
      }
    }
    MacroPath.push_back(start);
    //debug
    cout<<"heuristic cost"<<endl;
    for(auto itr : PlanWorkSpace){
      for(auto itrr : itr.TransLine){
        cout<<itrr.heuristic<<", ";
      }cout<<endl;
    }cout<<endl<<endl;

    //debug
    cout<<"Result set priority"<<endl;
    i=0;
    for(auto itr : PlanWorkSpace){
      cout<<"SurfaceID = ["<<i++<<"]___";
      for(auto itrr : itr.TransLine){
        cout<<itrr.PriorityPairNo<<", ";
      }
      cout<<endl;
    }

    //Prior Line set to front of PlanWorkspace.TransLine
    for(i=0;i<PlanWorkSpace.size();i++){
      for(int j=0;j<PlanWorkSpace.at(i).TransLine.size();j++){
        if(PlanWorkSpace.at(i).TransLine.at(j).PriorityPairNo == 1){
          TransLine PriorTL = PlanWorkSpace.at(i).TransLine.at(j);
          PlanWorkSpace.at(i).TransLine.at(j) = PlanWorkSpace.at(i).TransLine.front();
          PlanWorkSpace.at(i).TransLine.front() = PriorTL;
        }
      }
    }
    //debug
    cout<<endl<<"Set Prior Line to front of PlanWorkspace.TransLine"<<endl;
    for(auto itr : PlanWorkSpace){
      for(auto itrr : itr.TransLine){
        cout<<itrr.PriorityPairNo<<", ";
      }cout<<endl;
    }

    //close make heuristic cost on line

  


  //execute Astar
    vector<priority_queue<Guide, vector<Guide>, greater<Guide>>> AstarRes;
    stack<RobotState> prepath, path, path2;
    RobotState top;

      AstarRes = Astar(goalflg, Rparam, PlanWorkSpace, start, goal);

      if(goalflg == true){
          cout<<"Comlete Astar and reached to goal"<<endl<<endl;
          prepath = MakePath(goal, AstarRes);
          while(prepath.empty() != true){
            path.push(prepath.top());
            prepath.pop();
          }
      }
    path2 = path;

    //debug distance per step(if changed plane not equal to 1)
    while(path2.size() != 1){
        top = path2.top();
        path2.pop();
        cout<<sqrt( pow(top.center.x-path2.top().center.x, 2) + pow(top.center.y-path2.top().center.y, 2) + pow(top.center.z-path2.top().center.z, 2) )<<endl;
    }
  // close execute Astar
  



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
    for(i=0;i<Mids.size();i++){
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
  

    return 0;
}
