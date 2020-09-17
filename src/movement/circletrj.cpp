#include "circletrj.h"
#include "trj_config.h"
#include "stdio.h"

using namespace std;

circleTrj::circleTrj()
{
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
}
circleTrj::circleTrj(double time,
                     double start_x,  double start_y, double start_z,
                     double center_x, double center_y, double center_z,
                     double turn_rad, double duration, int facing)
{
    this->start_time = time;
    this->startx = start_x;
    this->starty = start_y;
    this->startz = start_z;
    this->centerx = center_x;
    this->centery = center_y;
    this->centerz = center_z;
    this->startyaw = atan2(starty-centery,startx-centerx);
    double dx=startx-centerx;
    double dy=starty-centery;
    double dz=startz-centerz;
    this->r = sqrt(dx*dx+dy*dy+dz*dz);
    this->est_t = duration;
    this->length_rad = turn_rad;
//    cout << "radious "  << r << endl;
//    cout << "startyaw " << startyaw << endl;
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
    facingsetup = facing;
}

void circleTrj::getPose(double time,
                        geometry_msgs::PoseStamped& pose)
{
    curr_time = time;
    if(est_flag == 0)
    {
        if(est_t>0.0 && est_t<1000.0)
        {
            av = length_rad / est_t;
            //cout << "angular velocity " << av << endl;
            est_flag = 1;
        }
    }
    if(est_flag == 1)
    {
        double dt=curr_time-start_time;
        double x=(cos(startyaw+(dt*av)))*r+centerx;
        double y=(sin(startyaw+(dt*av)))*r+centery;
        double z=centerz;
        Quaterniond q;
        if(facingsetup==CIRCLE_TRJ_FACING_CENTER){
          double yaw = atan2(centery-y,centerx-x);
          q = rpy2Q(Vec3(0,0,yaw));
        }
        if(facingsetup==CIRCLE_TRJ_FACING_FIXED){
          double yaw = 0;
          q = rpy2Q(Vec3(0,0,yaw));
        }
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
}

void circleTrj::getEnding(double &x, double &y, double &z, double &yaw)
{
    x=(cos(startyaw+length_rad))*r+centerx;
    y=(sin(startyaw+length_rad))*r+centery;
    z=centerz;
    yaw = atan2(centery-y,centerx-x);
}

int circleTrj::finished()
{
    if((curr_time-start_time)>=est_t)
    {return 1;}
    else
    {return 0;}
}

void circleTrj::setVerticalVelocityLimit(double v)
{
    vv_limit = v;
}

void circleTrj::setHorizonVelocityLimit(double v)
{
    hv_limit = v;
}

void circleTrj::setAngularSpeedRadLIMIT(double w)
{
    av_limit = w;
}
