#include "RobotState.h"
#include "Guide.cpp"
#include "3dtransform.cpp"

RobotState::RobotState(){
    this->center.x = 0;
    this->center.y = 0;
    this->center.z = 0;
    this->theta = -1;
    this->step = -1;
    this->SurfaceID = -1;
    this->score = 0;
    this->x = &this->center.x; y = &this->center.y; z = &this->center.z;
};

RobotState::RobotState(double inx, double iny, double inz, int intheta, int inplane){
    this->center.x = inx;
    this->center.y = iny;
    this->center.z = inz;
    this->theta = intheta;
    this->step = 0;
    this->SurfaceID = inplane;
    this->x = &this->center.x; y = &this->center.y; z = &this->center.z;
};

//copy constructer
RobotState::RobotState(const RobotState& cprobotstate){
    this->center = cprobotstate.center;
    this->theta = cprobotstate.theta;
    this->step = cprobotstate.step;
    this->SurfaceID = cprobotstate.SurfaceID;
    this->score = cprobotstate.score;
    this->x = &this->center.x; y = &this->center.y; z = &this->center.z;
};
RobotState& RobotState::operator=(const RobotState& cprobotstate){
    if(this == &cprobotstate)return *this;
    this->center = cprobotstate.center;
    this->theta = cprobotstate.theta;
    this->step = cprobotstate.step;
    this->SurfaceID = cprobotstate.SurfaceID;
    this->score = cprobotstate.score;
    this->NodeNo = cprobotstate.NodeNo;
    this->x = &this->center.x; y = &this->center.y; z = &this->center.z;

    return *this;
};

bool RobotState::operator==(const RobotState& cprobotstate){
    if( (this->center.x == cprobotstate.center.x) &&
        (this->center.y == cprobotstate.center.y) &&
        (this->center.z == cprobotstate.center.z) &&
        (this->theta == cprobotstate.theta) &&
        (this->SurfaceID == cprobotstate.SurfaceID) &&
        (this->NodeNo == cprobotstate.NodeNo)
    )return true;
    else return false;
};

void RobotState::MomentEdge2D(RobotParam& Rparam){
    //FR
    fr.x = this->center.x + cos(this->theta*(M_PI/180))*(Rparam.l/2) -sin(this->theta*(M_PI/180))*(-Rparam.w/2);
    fr.y = this->center.y + sin(this->theta*(M_PI/180))*(Rparam.l/2) +cos(this->theta*(M_PI/180))*(-Rparam.w/2);
    fr.z = this->center.z;

    //FL
    fl.x = this->center.x + cos(this->theta*(M_PI/180))*(Rparam.l/2) -sin(this->theta*(M_PI/180))*(Rparam.w/2);
    fl.y = this->center.y + sin(this->theta*(M_PI/180))*(Rparam.l/2) +cos(this->theta*(M_PI/180))*(Rparam.w/2);
    fl.z = this->center.z;

    //RL
    rl.x = this->center.x + cos(this->theta*(M_PI/180))*(-Rparam.l/2) -sin(this->theta*(M_PI/180))*(Rparam.w/2);
    rl.y = this->center.y + sin(this->theta*(M_PI/180))*(-Rparam.l/2) +cos(this->theta*(M_PI/180))*(Rparam.w/2);
    rl.z = this->center.z;
        
    //RR
    rr.x = this->center.x + cos(this->theta*(M_PI/180))*(-Rparam.l/2) -sin(this->theta*(M_PI/180))*(-Rparam.w/2);
    rr.y = this->center.y + sin(this->theta*(M_PI/180))*(-Rparam.l/2) +cos(this->theta*(M_PI/180))*(-Rparam.w/2);
    rr.z = this->center.z;

    Front.begin = fr; Front.end = fl;
    Rear.begin = rr;  Rear.end = rl;
    Left.begin = fl;  Left.end = rl;
    Right.begin = fr; Right.end = rr;

    this->vertex.clear();
    this->vertex.push_back(fr);
    this->vertex.push_back(fl);
    this->vertex.push_back(rl);
    this->vertex.push_back(rr);

    this->edges.clear();
    this->edges.push_back(Front);
    this->edges.push_back(Rear);
    this->edges.push_back(Left);
    this->edges.push_back(Right);
};

void RobotState::MomentEdge3D(RobotParam& Rparam, trans& offsetrotate){
    Point buff = offsetrotate.offset;
    offsetrotate.offset = this->center;
    this->MomentEdge2D(Rparam);
    
    fr = offsetrotate.Rotate(fr) - offsetrotate.offset + buff;
    fl = offsetrotate.Rotate(fl) - offsetrotate.offset + buff;
    rl = offsetrotate.Rotate(rl) - offsetrotate.offset + buff;
    rr = offsetrotate.Rotate(rr) - offsetrotate.offset + buff;

    Front.begin = fr; Front.end = fl;
    Rear.begin = rr;  Rear.end = rl;
    Left.begin = fl;  Left.end = rl;
    Right.begin = fr; Right.end = rr;

    this->vertex.clear();
    this->vertex.push_back(fr);
    this->vertex.push_back(fl);
    this->vertex.push_back(rl);
    this->vertex.push_back(rr);

    this->edges.clear();
    this->edges.push_back(Front);
    this->edges.push_back(Rear);
    this->edges.push_back(Left);
    this->edges.push_back(Right);
    offsetrotate.offset = buff;
};

RobotState& RobotState::update(RobotState& nex, RobotParam& p, double omega){
//generate next point
    //x(t+1) = x(t)+vt*Δt*cosθt
    //y(t+1) = y(t)+vt*Δt*sinθt
    //θ(t+1) = θ(t)+ωt*Δt
    //plane(t+1) = plane(t)

    nex.theta = this->theta+omega;
    if(nex.theta <= -180) nex.theta += 360;
    else if(nex.theta>180) nex.theta -=360;

    nex.center.x = this->center.x + p.v*cos(nex.theta*(M_PI/180));
    nex.center.y = this->center.y + p.v*sin(nex.theta*(M_PI/180));
    //nex.centor.z = this->centor.z;

    nex.step = this->step + p.v;
    nex.SurfaceID = this->SurfaceID;

    this->MomentEdge2D(p);
    
    //cout<<this->edges.size()<<"___";
    nex.edges = this->edges;

    
    return nex;
};

bool RobotState::checkreach(RobotState& another){
    
    if( ( this->SurfaceID == another.SurfaceID )
        && ( this->epstheta > abs(another.theta-this->theta) 
        && ( this->epsxy > sqrt( pow(another.center.x-this->center.x, 2) + pow(another.center.y-this->center.y, 2)))
        )
      ){
        return true;
    }
    else return false;
};


int RobotState::checkalready(priority_queue<Guide, vector<Guide>, greater<Guide>>& alreadylist){
    flg = -1, BuffNodeNo=-1;
    //flg -1 リスト内に存在しない
    //flg  0 リスト内に存在する  &&  既存のトータルコストの方が小さい(なにもしない)
    //flg  1 リスト内に存在する  &&  既存のトータルコストよりも小さい(要素の削除)
    //flg  2 リスト内に存在する　&&  既存のトータルコストよりも小さい　&&  リスト内のスタート地点(上書きしてはいけない)
    //0,2の時、何もしない。1の時だけ要素の削除
    Guide buff;
    priority_queue<Guide, vector<Guide>, greater<Guide>> anotherlist;

    while(alreadylist.empty() != true){
        buff = alreadylist.top();
        alreadylist.pop();
        if(this->checkreach(buff.current) == true){
            flg = 0;
            if(this->score < buff.current.score){
                flg = 1;
                if(buff.start == true)flg = 2;
            }
        }
        if(flg != 1){anotherlist.push(buff);}
        else{this->NodeNo = buff.current.NodeNo;}
    }
    alreadylist = anotherlist;
    return flg;
};




RobotState::~RobotState(){
};
