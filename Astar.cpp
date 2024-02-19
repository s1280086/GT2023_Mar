#ifndef A_STAR
#define A_STAR

#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <fstream>

//for sin() cos() tan()
//#define _USE_MATH_DEFINES
//#include <math.h>
using namespace std;

//#include "RobotParam.h"
//#include "RobotState.h"
#include "obstacle.cpp"
#include "Guide.cpp"
//#include "Surface.cpp"

vector<priority_queue<Guide, vector<Guide>, greater<Guide>>> Astar(bool& goaled, RobotParam p, vector<Surface>& WorkSpace, RobotState& start, RobotState& goal){
    priority_queue<Guide, vector<Guide>, greater<Guide>> OpenList;
    vector<priority_queue<Guide, vector<Guide>, greater<Guide>>> ClosedList(WorkSpace.size(), priority_queue<Guide, vector<Guide>, greater<Guide>>());
    priority_queue<Guide, vector<Guide>, greater<Guide>> pqguide;

    //vector<rectangle> obs; //= PlaneObs;

    bool collision = false;
    int CheckOpen = -1, CheckClosed = -1, nextfloor = -1;

    Guide curpre, nex;
    curpre.current = start; curpre.previous = start; curpre.start = true;

    Line traj, BeforeLine, AfterLine;//for check change floor
    traj.begin = curpre.previous.center; traj.end = curpre.current.center;

    
    //set cost
    /* A*のコスト= startからcurpre.currentまでの距離  
               + 
               + 
    */
    curpre.current.score = curpre.current.step;
    if(curpre.current.SurfaceID == goal.SurfaceID){
        curpre.current.score += goal.center.distance(start.center);
    }
    else{
        curpre.current.score += WorkSpace.at(curpre.current.SurfaceID).TransLine.front().Pair.first.mid.distance(nex.current.center);
    }
    curpre.current.score += WorkSpace.at(curpre.current.SurfaceID).TransLine.front().heuristic;

    OpenList.push(curpre);

    goal.epsxy = 0.07017;
    goal.epstheta = 15;

    //for debug
    int cnt=0;
    //curpre.current.NodeNo = cnt;

    while(OpenList.size() != 0){
        //for debug
        if(cnt%1000 == 0)std::cout<<cnt<<", ";
        //cnt++; 

        curpre = OpenList.top();
        OpenList.pop();


        //check goal
        if(goal.checkreach(curpre.current) == true){
            cout<<"reached goal\n";
            cout<<"goal coordinate = "<<*curpre.current.x<<", "<<*curpre.current.y<<", "<<*curpre.current.z<<", "<<curpre.current.theta<<", planeNo "<<curpre.current.SurfaceID<<endl;
            goal = curpre.current;
            goaled = true;
            ClosedList.at(curpre.current.SurfaceID).push(curpre);
            break;
        }
        //OpenListから取り出したGuideをClosedListに投げ込む
        ClosedList.at(curpre.current.SurfaceID).push(curpre);


        nex.previous = curpre.current;
        curpre.current.MomentEdge2D(p);

        cout<<" "<<curpre.current.SurfaceID<<" "<<curpre.current.score<<endl;

        // +30° 0° -30°
        for(int i = 1; i>-2; i--){
            //generate next point
            nex.current = curpre.current.update(nex.current, p, p.omega*i);
            
            traj.begin = nex.previous.center;
            traj.end = nex.current.center;

            //Surfaceが変わるかチェック
            for(auto itrPairLine : WorkSpace.at(nex.current.SurfaceID).TransLine){
                if( traj.judgecross(itrPairLine.Pair.first) ){
                    BeforeLine = itrPairLine.Pair.first;
                    AfterLine = itrPairLine.Pair.first.SwitchLine(itrPairLine.Pair, nextfloor);
                    break;
                }
                if(nextfloor != -1)break;
            }

            if(nextfloor == -1){
                //nex.currentと同一平面上にあるobstacleのみcheckcoll
                for(auto itrObs : WorkSpace.at(nex.current.SurfaceID).PlnObs){
                    if(itrObs.checkcoll(nex.current.edges) == true){
                        collision = true;
                        break;
                    }
                }
            }
            else{//Transition floor(if need)
                WorkSpace.at(nex.current.SurfaceID).FrameTrans(nex.current, BeforeLine, AfterLine);
                nex.current.SurfaceID = nextfloor;
            }


            //set Cost
            if(nex.current.SurfaceID == goal.SurfaceID){//on goal surface
                nex.current.score = nex.current.step + goal.center.distance(nex.current.center);
            }
            else{//in other surface(distance from prior transition line)
                nex.current.score = nex.current.step + WorkSpace.at(nex.current.SurfaceID).TransLine.front().Pair.first.mid.distance(nex.current.center);
            }
            nex.current.score += WorkSpace.at(nex.current.SurfaceID).TransLine.front().heuristic;
            

            //push to List
            if(collision == false){
                CheckOpen = nex.current.checkalready(OpenList);
                CheckClosed = nex.current.checkalready(ClosedList.at(nex.current.SurfaceID));

                //OpenListにあるかつnexの方がscoreが小さい(return 1) → OpenListに置き換える(削除はcheckalready内で行う)
                //ClosedListにあるかつnexの方がscoreが小さい(return 1) → ClosedListから削除してOpenListに追加
                if((CheckOpen == 1) || (CheckClosed == 1)){
                    OpenList.push(nex);}
                
                //OpenListにもClosedListにも無い(return -1) → OpenListに追加
                if((CheckOpen == -1) && (CheckClosed == -1)){
                    nex.current.NodeNo = ++cnt;
                    OpenList.push(nex);}
            }
            else{/*cout<<"collision == true"<<endl;*/}
            

            //reset "collision check" and "next floor No"
            collision = false;
            nextfloor = -1;
            CheckOpen = -1, CheckClosed = -1;
        }

    }
    
    cout<<"ClosedList->size() = ";
    for(int i=0;i<ClosedList.size();i++){
        cout<<ClosedList.at(i).size()<<"  ";
    }
    cout<<endl;
    return ClosedList;
};

stack<RobotState> MakePath(RobotState& goal, vector<priority_queue<Guide, vector<Guide>, greater<Guide>>> pq){

    Guide curpre, buff;
    stack<RobotState> path;
    priority_queue<Guide, vector<Guide>, greater<Guide>> hoge;

    cout<<"Making Path ... "<<endl;

    curpre.current = goal;
    while(pq.at(goal.SurfaceID).empty() != true){
        buff = pq.at(goal.SurfaceID).top();
        pq.at(goal.SurfaceID).pop();
        
        if(curpre.current == buff.current){
            curpre = buff;
        }
        //cout<<"goal.pqsize = "<<pq.at(goal.planeno).size()<<endl;
        hoge.push(buff);
    }
    //priority_queue.clear()の代わり↓
    pq.at(goal.SurfaceID) = priority_queue<Guide, vector<Guide>, greater<Guide>>();
    pq.at(goal.SurfaceID) = hoge;

    cout<<"goal.pqsize = "<<pq.at(goal.SurfaceID).size()<<endl;

    path.push(curpre.current);
    cout<<"find goal state"<<endl;

    int hogehoge = 0;
    while(curpre.start != true){
        hoge = priority_queue<Guide, vector<Guide>, greater<Guide>>();
        int SurfaceID = curpre.previous.SurfaceID;
        int cnt = 0;
        

        //curpre.previous == hoge.currentとなる hogeをpqから探す。
        while(pq.at(SurfaceID).empty() != true){
            hogehoge++;
            
            cnt++;
            buff = pq.at(SurfaceID).top();
            pq.at(SurfaceID).pop();

            cout<<cnt<<", "<<buff.current.center.distance(curpre.previous.center)<<endl;

            //curpre.previousが見つからない？
            //"curpre.previous"="hoge.current"におけるノードの上書きによってcheckreach外に入ってしまう
            //対策は？→ノード番号を付ける
            
            curpre.previous.epstheta = 0.05;
            if(curpre.previous.checkreach(buff.current) == true){
                hogehoge = 0;
            //if(curpre.previous == buff.current){
                curpre = buff;
                path.push(buff.current);
                cout<<"find previous";
                cout<<" "<<curpre.previous.SurfaceID<<" "<<curpre.current.SurfaceID<<" cost = "<<curpre.previous.score;
                cout<<", "<<curpre.previous.center.x<<" "<<curpre.previous.center.y<<" "<<curpre.previous.theta;
                cout<<", found in "<<pq.at(curpre.current.SurfaceID).size()<<"/"<<pq.at(curpre.current.SurfaceID).size() + cnt<<endl;
            }
            /*
            if(curpre.previous.NodeNo == buff.current.NodeNo){
                curpre = buff;
                path.push(buff.current);
                cout<<"find previous";
                cout<<" "<<curpre.previous.SurfaceID<<" "<<curpre.current.SurfaceID<<" cost = "<<curpre.previous.score;
                cout<<", "<<curpre.previous.center.x<<" "<<curpre.previous.center.y<<" "<<curpre.previous.theta;
                cout<<", found in "<<pq.at(curpre.current.SurfaceID).size()<<"/"<<pq.at(curpre.current.SurfaceID).size() + cnt<<endl;
            }*/
            
            //cout<<buff.current.NodeNo<<endl;
            hoge.push(buff);
        }
        if(hogehoge>500)break;
        
        pq.at(SurfaceID) = priority_queue<Guide, vector<Guide>, greater<Guide>>();
        pq.at(SurfaceID) = hoge;
        cout<<"end pq";
    }

    cout<<"Complete path.size() = "<<path.size()<<endl;
    return path;
}


#endif
