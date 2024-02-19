//RobotParam.h
#ifndef ROBOTPARAM
#define ROBOTPARAM

using namespace std;

/*
  This struct set Robot Parameta.
  For example
    v = 0.1 m/s
    t = 1s (1step for 1second)
    omega = 30 deg/s
*/

struct RobotParam{
    public:
        double v, t, omega;
        double w, l, h;
};

#endif