#ifndef CIRCLETRY_H
#define CIRCLETRY_H

#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include "../utils/kinetic_math.h"
#include "trj_config.h"


class circleTrj
{
public:
    circleTrj();
    circleTrj(double time,
              double start_x,  double start_y, double start_z,
              double center_x, double center_y, double center_z,
              double turn_rad, double duration, int facing);
    void getPose(double time, geometry_msgs::PoseStamped& pose);
    void setVerticalVelocityLimit(double v);
    void setHorizonVelocityLimit(double v);
    void setAngularSpeedRadLIMIT(double w);
    int  finished(void);
    void getEnding(double& x, double& y, double& z, double& yaw);
private:
    int    facingsetup;
    double centerx,centery,centerz;
    double startx, starty, startz, startyaw;
    double length_rad;
    double r;//radious
    double av;//angular velocity

    double hv_limit,vv_limit,av_limit;
    double start_time;
    double curr_time;
    double est_t;
    int est_flag;
};



#endif // CIRCLETRY_H
