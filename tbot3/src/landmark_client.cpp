/* ---------------ROS CLIENT-----------------------------------------
## Client is the one who is asking for the services provided by the Node

             ___Important_Points___
## will printout the closest landmark
## print out the distance of each landmark
## don't need to use the ros::spin(), bcz landmark_client just prints
   the data and then stop 
## wait for duration is used to synchronize the client and the server:
    useful when sometime client execute before the server
*/
#include <string>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "tbot3_msgs/GetClosest.h"
#include "tbot3_msgs/GetDistance.h"
#include "tbot3_msgs/GetClosestRequest.h"


int main(int argc, char**argv){
    // initialize the ros node with name "landmark_client"
    ros::init(argc, argv, "landmark_client");
    ros::NodeHandle nh;

    //only added to make the ros::waitForExistence to work
    while(ros::Time::now().isZero()){
        ros::spinOnce();
    }

    //create the service client and hook it with the available service of the node here "get_closest"
    ros::ServiceClient get_closest = nh.serviceClient<tbot3_msgs::GetClosest>("get_closest");
    ros::ServiceClient get_distance = nh.serviceClient<tbot3_msgs::GetDistance>("get_distance");

    //implement the waitForExistence to synchronize the client and server
    if (!get_closest.waitForExistence(ros::Duration(5))){
        ROS_ERROR("get_closest server did not come up in 5 seconds!");
        return 1;
    }

    if(!get_distance.waitForExistence(ros::Duration(5))){
        ROS_ERROR("get_distance server did not come up in 5 seconds");
        return 1;
    }
    //call the service via client
    tbot3_msgs::GetClosestRequest req;
    tbot3_msgs::GetClosestResponse res;
    if (!get_closest.call(req, res)){
        ROS_ERROR("Failed to call GetClosest!");
    }

    std::cout<< "Closest Landmark: "<<res.name<<std::endl;
    //for closest distance create vector of landmarks
    std::vector<std::string> landmarks_list = {"Construction_barrel","Dumpster", "Number1", "Jersey_barrier"};

    for(const std::string &name: landmarks_list){
        tbot3_msgs::GetDistanceRequest dist_req;
        dist_req.name = name;
        tbot3_msgs::GetDistanceResponse dist_res;
        get_distance.call(dist_req, dist_res);
        std::cout<<name<<" : "<<dist_res.distance<<std::endl;

    }
    return 0;

}

