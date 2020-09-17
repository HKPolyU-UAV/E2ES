#ifndef GENERALMOVE_H
#define GENERALMOVE_H

#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <utils/include/kinetic_math.h>
#include "trj_config.h"


class generalMove
{
public:
    generalMove();
    generalMove(double time,
                double start_x, double start_y, double start_z, double start_yaw_rad,
                double end_x, double end_y, double end_z, double end_yaw,
                double duration);
    void getPose(double time,
                 geometry_msgs::PoseStamped& pose);
    void setVerticalVelocityLimit(double v);
    void setHorizonVelocityLimit(double v);
    void setAngularSpeedRadLIMIT(double w);
    int  finished(void);
private:
    double endx, endy, endz, endyaw;
    double startx, starty, startz, startyaw;
    double hv_limit,vv_limit,av_limit;
    double start_time;
    double curr_time;
    double vx,vy,vz,v_yaw;
    double est_t;
    int est_flag;
};



#endif // GENERALMOVE_H
