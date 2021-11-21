#include "landmarks/landmarks.h"

Landmarks::Landmarks ():loop_rate(10) {
    op_sub = n.subscribe("object_positions_in",10, &Landmarks::callback, this);
    op_pub = n.advertise<vortex_msgs::ObjectPosition>("object_positions_out",10);
    service = n.advertiseService("send_positions", &Landmarks::send_pos, this);
}

void Landmarks::callback(vortex_msgs::ObjectPosition objPos){
    objectPositions[objPos.objectID] = objPos.position;
    op_pub.publish(objPos);            
}

void Landmarks::execute(){
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Landmarks::printMap(std::map<std::string,geometry_msgs::Point> objectsMap){
    for(auto elem : objectsMap){
        ROS_INFO("ID: %s", elem.first.c_str());
        ROS_INFO("position: %f,%f,%f",elem.second.x,elem.second.y,elem.second.z);
            
    }
}

bool Landmarks::send_pos(landmarks::request_position::Request &req, landmarks::request_position::Response &res){
    res.pos = Landmarks::objectPositions[req.ID];
    //for testing:
    // geometry_msgs::Point p;p.x = 1;p.y = 1;p.z = 1;
    // res.pos=p;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"landmarks");
    Landmarks lm;
    lm.execute();
}