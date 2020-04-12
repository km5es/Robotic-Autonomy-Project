#include "ros/ros.h"
#include "human_prob_motion/HumanProbMotion.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_motion_client");

  if (argc != 16)
  {
    //ROS_INFO("usage: human_motion_client vli vri thk posinx posiny");
    ROS_INFO("usage: human_motion_client Sx0 Sx1 Su0 Su1 Tx0 Tx1 vri vli thk Tm0 Tm1 TS0 TS1 TS2 TS3");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<human_prob_motion::HumanProbMotion>("human_prob_motion");

  human_prob_motion::HumanProbMotion srv;
  // position of robot
  srv.request.Sx0 = atof(argv[1]);
  srv.request.Sx1 = atof(argv[2]);
  // motion of robot
  srv.request.Su0 = atof(argv[3]);
  srv.request.Su1 = atof(argv[4]);
  // position of target
  srv.request.Tx0 = atof(argv[5]);
  srv.request.Tx1 = atof(argv[6]);
  // motion of target
  //srv.request.Tu0 = atof(argv[7]);
  //srv.request.Tu1 = atof(argv[8]);
  srv.request.vri = atof(argv[7]);
  srv.request.vli = atof(argv[8]);
  srv.request.thk = atof(argv[9]);
  // Tm and TS
  srv.request.Tm0 = atof(argv[10]);
  srv.request.Tm1 = atof(argv[11]);
  srv.request.TS0 = atof(argv[12]);
  srv.request.TS1 = atof(argv[13]);
  srv.request.TS2 = atof(argv[14]);
  srv.request.TS3 = atof(argv[15]);
 
  
  if (client.call(srv))
  {
    ROS_INFO("Response: ");
    // position of robot
    ROS_INFO("Position of robot: %lf, %lf", (float)srv.response.Sxr0, (float)srv.response.Sxr1);
    // position of target and orientation
    ROS_INFO("Position of target: %lf, %lf", (float)srv.response.Txr0, (float)srv.response.Txr1);
    ROS_INFO("Orientation of target: %lf", (float)srv.response.thk);
    ROS_INFO("Tm: %lf, %lf", (float)srv.response.Tmr0, (float)srv.response.Tmr1);
    ROS_INFO("TS: %lf, %lf, %lf, %lf", (float)srv.response.TSr0, (float)srv.response.TSr1, (float)srv.response.TSr2, (float)srv.response.TSr3);
    
  }
  else
  {
    ROS_ERROR("Failed to call service human_prob_motion");
    return 1;
  }
 
  return 0;
}
