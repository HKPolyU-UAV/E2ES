#include "generalmove.h"
#include "stdio.h"

using namespace std;

generalMove::generalMove()
{
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
}
generalMove::generalMove(double time,
                         double start_x, double start_y, double start_z, double start_yaw_rad,
                         double end_x, double end_y, double end_z, double end_yaw_rad,
                         double duration)
{
    this->start_time = time;
    this->startx = start_x;
    this->starty = start_y;
    this->startz = start_z;
    this->startyaw = start_yaw_rad;
    this->endx = end_x;
    this->endy = end_y;
    this->endz = end_z;
    this->endyaw = end_yaw_rad;
    this->est_t = duration;
    vv_limit = DEFALUT_VV_LIMIT;
    hv_limit = DEFAULT_HV_LIMIT;
    av_limit = DEFAULT_AV_LIMIT;
    est_flag = 0;
}

void generalMove::getPose(double time,
                          geometry_msgs::PoseStamped& pose)
{
    curr_time = time;
    if(est_flag == 0)
    {
        if(est_t>0.0 && est_t<1000.0)
        {
//            cout << "start_yaw" << this->startyaw << endl;
//            cout << "end_yaw" << this->endyaw << endl;
            double hv = (sqrt(pow((endx-startx),2)+pow((endy-starty),2)))/est_t;
            double vv = (sqrt(pow((endz-startz),2)))/est_t;
            if (startyaw>=M_PI)  startyaw-=2*M_PI;
            if (startyaw<=-M_PI) startyaw+=2*M_PI;
            if (endyaw>=M_PI)    endyaw-=2*M_PI;
            if (endyaw<=-M_PI)   endyaw+=2*M_PI;
            double d_yaw = endyaw - startyaw;
            if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
            if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
            double av = fabs(d_yaw/est_t);
            if(hv > hv_limit){std::cout << "check horizen speed limit" << std::endl;}
            if(vv > vv_limit){std::cout << "check vertical speed limit" << std::endl;}
            if(av > av_limit){std::cout << "check angular speed limit" << std::endl;}
            if(hv < hv_limit && vv < vv_limit && av < av_limit)
            {
                vx = (endx-startx)/est_t;
                vy = (endy-starty)/est_t;
                vz = (endz-startz)/est_t;
                v_yaw = (d_yaw)/est_t;
                est_flag = 1;
            }
        }
    }
    if(est_flag == 1)
    {
        double dt=curr_time-start_time;
        pose.pose.position.x=startx+dt*vx;
        pose.pose.position.y=starty+dt*vy;
        pose.pose.position.z=startz+dt*vz;
        Quaterniond q = rpy2Q(Vec3(0,0,startyaw+dt*v_yaw));
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
    }
}

int generalMove::finished()
{
    if((curr_time-start_time)>=est_t)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void generalMove::setVerticalVelocityLimit(double v)
{
    vv_limit = v;
}

void generalMove::setHorizonVelocityLimit(double v)
{
    hv_limit = v;
}

void generalMove::setAngularSpeedRadLIMIT(double w)
{
    av_limit = w;
}
