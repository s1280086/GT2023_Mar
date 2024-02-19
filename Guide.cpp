#ifndef GUIDE
#define GUIDE

#include "RobotState.h"

class Guide{
    public:
        bool start, goal;
        RobotState current, previous;
        int step;

        Guide(){
            this->start=false;
            this->goal=false;
    };
        Guide(const Guide& cpguide){
        this->current = cpguide.current;
        this->previous = cpguide.previous;
        this->step = cpguide.step;
        this->start = cpguide.start;
        this->goal = cpguide.goal;
    };

    Guide& operator=(const Guide& cpguide){
        if(this == &cpguide)return *this;
        this->current = cpguide.current;
        this->previous = cpguide.previous;
        this->step = cpguide.step;
        this->start = cpguide.start;
        this->goal = cpguide.goal;
        return *this;
    };
    bool operator==(const Guide& cpguide){
        if((this->current == cpguide.current)&&
            (this->previous == cpguide.previous)
        )return true;
        return false;
    };
    
    /*
    bool operator<(const Guide& cpguide1){
        return current.step < cpguide1.current.step;
    };
    */
    

    bool operator > (const Guide& cpguide) const{
        if(cpguide.current.score < current.score)return true;
        else{return false;}
    };
    
    
    
};

#endif