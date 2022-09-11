#include<string>
#include<math.h>
#include<map>

#include "ros/ros.h"
#include "tbot3_msgs/GetClosest.h"
#include "tbot3_msgs/GetDistance.h"
#include "nav_msgs/Odometry.h"


struct Landmark
{
    /* attributes of landmark list */
    std::string name;  // name of landmark, x & y position
    double x;          
    double y;
};

class LandmarkMonitor
{
private:
    /* attributes of the class*/
    std::map<std::string, Landmark> landmark_list;
    double robot_pos_x;    // get these values by listening odom topic
    double robot_pos_y;    // for listening subscribe to the odom topic via subscribe functionality

public:
    LandmarkMonitor();
    ~LandmarkMonitor();

    /* class methods to implement service callback functions*/
    bool GetClosest_callback(tbot3_msgs::GetClosest::Request& request, tbot3_msgs::GetClosest::Response& response){
        ROS_INFO("GetClosest Service Called");

        double close_distance = -1;
        std::string close_landmark("");

        for(const auto& x: landmark_list){
            const Landmark& landmark = x.second;
            // get position of the landmark
            double landmark_pos_x = landmark.x;
            double landmark_pos_y = landmark.y;

            //compute robot distance from landmark
            double dx = landmark_pos_x - robot_pos_x;
            double dy = landmark_pos_y - robot_pos_y;

            // euclidian distance
            double dd = sqrt(dx*dx + dy*dy);

            if(close_distance == -1 || dd< close_distance){
                close_distance = dd;
                close_landmark = x.first;
            }
        }

        response.name = close_landmark;
        return true;
    }

    bool GetDistance_callback(tbot3_msgs::GetDistance::Request& request, tbot3_msgs::GetDistance::Response& response){
        ROS_INFO("GetDistance Service Called for [%s]", request.name.c_str());

        //check landmark exit or not
        if(landmark_list.find(request.name) == landmark_list.end()){
            ROS_ERROR("Unknown Landmark [%s]", request.name.c_str());
            return false;
        }
        //if exit  get landmark_x_pos, landmark_y_pos
        const Landmark &landmark_variable = landmark_list[request.name];
        double landmark_x_pos = landmark_variable.x;
        double landmark_y_pos = landmark_variable.y;

        // compute robot distance from the landmark positions
        double dx = landmark_x_pos - robot_pos_x;
        double dy = landmark_y_pos - robot_pos_y;

        //return the response in the form of euclidian distance
        response.distance = sqrt(dx*dx + dy*dy);
        return true; // to tell that call has been sucessful
    }


    /* class methods to implemet subscribe callbcak function*/
    void Odom_callback(const nav_msgs::OdometryConstPtr& msg){
        robot_pos_x = msg->pose.pose.position.x;
        robot_pos_y = msg->pose.pose.position.y;
    }

};

LandmarkMonitor::LandmarkMonitor()
    :landmark_list(), robot_pos_x{0}, robot_pos_y{0}{
    landmark_list = {
        {"Construction_barrel", {"Constrcution_barrel", 5.20314, 3.591}},
        {"Dumpster", {"Dumpster", 3.190, -4.9259}},
        {"Jersey_barrier", {"Jersey_barrier", 0.31978, 1.38024}},
        {"Number1", {"Number1", 0.359466, 3.82409}}
    };
}

LandmarkMonitor::~LandmarkMonitor()
{
}

int main(int argc, char**argv){
    //initialize the ros node with name "landmark_monitor"
    ros::init(argc, argv, "landmark_monitor");
    ros::NodeHandle nh;

    //instance of the class
    LandmarkMonitor monitor;

    //create ros services and hook them up with above defined service callback functions
    ros::ServiceServer get_closest = nh.advertiseService("get_closest", &LandmarkMonitor::GetClosest_callback, &monitor);
    ros::ServiceServer get_distance = nh.advertiseService("get_distance", &LandmarkMonitor::GetDistance_callback, &monitor);

    //create subscriber and hook them up with above defined subscribe callback function
    ros::Subscriber sub = nh.subscribe("odom",1, &LandmarkMonitor::Odom_callback,&monitor);

    ros::spin();
    return 0;
}


/*
##  callback functions are the one that will implement the service
##  why using class :
    if there is a state that is shared btw main and callback function, 
    for this that state has to be the global variable, to avoid this thing,
    we are implementing the service callback functions as method of the class

