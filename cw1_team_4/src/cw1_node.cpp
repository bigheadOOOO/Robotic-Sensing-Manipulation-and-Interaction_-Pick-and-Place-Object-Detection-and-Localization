#include <cw1_class.h>

int main(int argc, char **argv){
  
  ros::init(argc,argv, "cw1_solution_node");
  ros::NodeHandle nh;

  // create an instance of the cw1 class
  cw1 cw_class(nh);
  
  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // loop rate in Hz
  ros::Rate rate(5);

  while (ros::ok()) {

    // spin and process all pending callbacks
    ros::spinOnce();
    // sleep to fulfill the loop rate
    rate.sleep();
  }
}

