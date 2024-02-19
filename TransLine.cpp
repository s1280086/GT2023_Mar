#ifndef TRANSLINE
#define TRANSLINE
#include "Line.cpp"
#include <iostream>
using namespace std;

class TransLine{
    public: 
      pair<Line, Line> Pair;
      int PriorityPairNo = -1;
      int PairNo = -1;
      double heuristic = 0;

      TransLine(){};
      TransLine(Line l1,Line l2, int inPairNo){
        this->Pair.first  = l1;
        this->Pair.second = l2;
        this->PairNo = inPairNo;
      };
      void SetPriority(const int inPriorityPairNo){
        if(this->PriorityPairNo == -1)PriorityPairNo = inPriorityPairNo;
      }
};

#endif