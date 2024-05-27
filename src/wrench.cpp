#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

struct WRENCH{
        double fx;
        double fy;
        double fz;
};

static ros::ServiceClient body_wrench_client;
static ros::Publisher wrench_data_pub;

static WRENCH applied_wrench;
static double mass = 11.2;     // Unit: kg
static gazebo_msgs::ApplyBodyWrench wrench; 

void applyDisturbance()
{
    wrench.request.body_name = "iris::base_link";
    wrench.request.reference_frame = "world"; //NED
    wrench.request.wrench.force.x = applied_wrench.fx;
    wrench.request.wrench.force.y = applied_wrench.fy;
    wrench.request.wrench.force.z = applied_wrench.fz; 
    // wrench.request.wrench.torque.z = 5.0;

    wrench.request.reference_point.x = 0.0;
    wrench.request.reference_point.y = 0.0;
    wrench.request.reference_point.z = 0.0;
    wrench.request.start_time = ros::Time::now();
    wrench.request.duration = ros::Duration(0.05);  // Duration of the disturbance
    body_wrench_client.call(wrench);

    std::cout<<"------------------ applied disturbances xyz ------------------"<<std::endl;
    std::cout<<"mass: "<<mass<<std::endl;
    std::cout<<"applied_wrench_fx: "<<wrench.request.wrench.force.x<<" N"<<" = "<<wrench.request.wrench.force.x/mass<<" ms^-2"<<std::endl;
    std::cout<<"applied_wrench_fy: "<<wrench.request.wrench.force.y <<" N"<<" = "<<wrench.request.wrench.force.y/mass<<" ms^-2"<<std::endl;
    std::cout<<"applied_wrench_fz: "<<wrench.request.wrench.force.z<<" N"<<" = "<<wrench.request.wrench.force.z/mass<<" ms^-2"<<std::endl;

    // wrench_data_pub.publish(wrench_data);
}

void mainspin_cb(const ros::TimerEvent& e)
{
    applyDisturbance();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbance");
    ros::NodeHandle nh("~");


    ros::Timer mainspin_timer = nh.createTimer(
        ros::Duration(1.0/50.0),
        &mainspin_cb
    );

    body_wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    wrench_data_pub = nh.advertise<geometry_msgs::Point>("/current_wrench",1);

    nh.getParam("wrench_x",  applied_wrench.fx);
    nh.getParam("wrench_y",  applied_wrench.fy);
    nh.getParam("wrench_z",  applied_wrench.fz);
    
    ros::spin();

    return 0;
}