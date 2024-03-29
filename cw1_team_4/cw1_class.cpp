/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

///////////////////////////////////////////////////////////////////////////////
// PCL: 3D geometry processing, declare own point (point cloud). 
// a large point cloud need more time and momery
// voxelgrid filter: reduce the num of points, save time and momery
cw1::cw1(ros::NodeHandle nh)
{
  /* class constructor */

  nh_ = nh;



  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);
  sub_task = nh_.subscribe("/r200/camera/depth_registered/points", 1, 
  &cw1::imageColorCallback, this);
  // sub_task3 = nh_.subscribe("/r200/camera/depth_registered/points", 1, 
  // &cw1::imageColorCallbackTask3, this);
  
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pub = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud", 1);
  
  pub1 = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud1", 1);
  pub2 = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud2", 1);
  pub3 = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud3", 1);

  pub11 = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud11", 1);
  pub22 = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud22", 1);
  pub33 = nh_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud33", 1);

  // define Orienation: 
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  orientation = tf2::toMsg(q_result);


  // task 3
  // set filter
  voxel_grid.setLeafSize(0.0025f, 0.0025f, 0.0025f); // Adjust voxel size as needed

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  
  // set parameters for pose
  geometry_msgs::Pose pose;
  pose.position.x = request.object_loc.pose.position.x;
  pose.position.y = request.object_loc.pose.position.y;
  pose.position.z = request.object_loc.pose.position.z;
  // define grasping as from abov
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);
  pose.orientation = grasp_orientation;

  // pick the object
  int success = pickObj(pose); 

  // drop the object
  pose.position.x = request.goal_loc.point.x;
  pose.position.y = request.goal_loc.point.y;
  pose.position.z = request.goal_loc.point.z+basket_length/2;
  success *= dropObj(pose);

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  return success;
}

///////////////////////////////////////////////////////////////////////////////
// task 2
bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  /* Define pose */
  geometry_msgs::Pose target_pose;
  // Orienation: 
  target_pose.orientation = orientation;
  
  /* Basic definition and initialization */
  bool success = true;
  float red;
  float green;
  float blue;
  bool flag = false;
  int num_locs = request.basket_locs.size();
  
  /* Main task loop */
  for(int index = 0; index < num_locs; index++)
  {
    // Set the position of the target pose
    target_pose.position = request.basket_locs[index].point;
    if(index == 0) 
    {
      target_pose.position.z = 0.5;
      success *= moveArm(target_pose);
    }
    target_pose.position.z = request.basket_locs[index].point.z+z_offset_obj+basket_length+approach_distance_basket;

    // move to target points
    success *= moveArm(target_pose);
    // get pointcloud
    image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::Duration(0.15).sleep();
    sub_flag_task2 = true;
    while (image_data_cloud->empty()) {
      ros::Duration(0.1).sleep(); // Sleep for 0.01 second
      ROS_WARN("callback image");
    } // stop store pointcloud information
    sub_flag_task2 = false; 

    
    /** Analyse the pointcloud data from camera **/
    
    /* Filt before anaysing */
    // Test: check the pointcloud in rviz
    if(!image_data_cloud->empty())
    {
      sensor_msgs::PointCloud2 cloud_msg;
      cloud_msg.header.frame_id = camera_frame;  
      pcl::toROSMsg(*image_data_cloud, cloud_msg);
      pub1.publish(cloud_msg);
    }
    /* Analyse the pointcloud */
    // int index = image_data_cloud->points.size();
    for (int i = 0; i < image_data_cloud->points.size(); i ++) 
    {
      // get normalized RGB values 
      red = ((float)image_data_cloud->points[i].r)/255.0;
      green = ((float)image_data_cloud->points[i].g)/255.0;
      blue = ((float)image_data_cloud->points[i].b)/255.0;

      // decide the color of ith point in the cloud
      if(abs(red - BLUE[0]) < 0.1 && abs(green - BLUE[1]) < 0.1 && abs(blue - BLUE[2]) < 0.1)
      {
        response.basket_colours.push_back("blue");
        ROS_INFO("-----------%s", response.basket_colours.back().c_str());
        flag = true;
        break;
      }else if(abs(red - RED[0]) < 0.1 && abs(green - RED[1]) < 0.1 && abs(blue - RED[2]) < 0.1)
      {
        response.basket_colours.push_back("red");
        ROS_INFO("-----------%s", response.basket_colours.back().c_str());
        flag = true;
        break;
      }else if(abs(red - PURPLE[0]) < 0.1 && abs(green - PURPLE[1]) < 0.1 && abs(blue - PURPLE[2]) < 0.1)
      {
        response.basket_colours.push_back("purle");
        ROS_INFO("-----------%s", response.basket_colours.back().c_str());
        flag = true;
        break;
      }
    }
    // when there is no target color in the pointcloud
    if(flag == false)
    {
      response.basket_colours.push_back("none");
      ROS_INFO("-----------None");
    }else{
      flag = false;
    }
  }


  ROS_INFO("ALL COLORS:");
  for (int index = 0; index < response.basket_colours.size(); index++)
  {
    ROS_INFO("%s ", response.basket_colours[index].c_str());
  }
  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}


void
cw1::imageColorCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  if (!sub_flag_task3 && !sub_flag_task2) return; // only start to store the pointcloud message after task begins
  ROS_WARN("Image color callback function---");
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Check if cloud_input_msg is valid
  if (cloud_input_msg->data.size() == 0) {
    ROS_ERROR("Input cloud message is empty.");
    return;
  }
  // Store RGB information from the camera
  std::unique_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*cloud_input_msg, *cloud);
  // Check if the PCLPointCloud2 is empty
  if (cloud->data.empty()) {
    ROS_ERROR("PCLPointCloud2 conversion failed or resulted in an empty cloud.");
    return;
  }
  // transform msg data into pointcloud data
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> image_data_cloud_origin(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud, *image_data_cloud_origin);
  // Check if the resulting PointCloud is empty
  if (image_data_cloud_origin->empty()) {
    ROS_ERROR("Conversion from PCLPointCloud2 to PointCloud<PointXYZRGB> resulted in an empty cloud.");
    return;
  }

   /* filter operations */ 
  if(sub_flag_task3)
  {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> image_data_cloud_downsize(new pcl::PointCloud<pcl::PointXYZRGB>);
    // filter out all green points in the cloud, store it in the global variable pointer image_data_cloud
   pcl::PointXYZRGB green_min_color(0, 100, 0);
    pcl::PointXYZRGB green_max_color(100, 255, 100);

    colorFilter(image_data_cloud_origin, image_data_cloud_downsize, green_min_color, green_max_color, true);
    // Check if the PCLPointCloud2 is empty
    if (image_data_cloud_downsize->empty()) {
      ROS_WARN("task3: Data is not correct. No green point the the cloud %ld", image_data_cloud_origin->size());
      return;
    }
    // downsize the cloud for faster and more efficient analysis with a filter
    voxel_grid.setLeafSize(0.0023f, 0.0023f, 0.0023f); // Set leaf size 
    voxel_grid.setInputCloud(image_data_cloud_downsize); 
    image_data_cloud_origin->clear();
  }
  else if(sub_flag_task2)
  {
    // downsize the cloud for faster and more efficient analysis with a filter
    voxel_grid.setLeafSize(0.02f, 0.02f, 0.02f); // Set leaf size 
    voxel_grid.setInputCloud(image_data_cloud_origin); 
    if (image_data_cloud_origin->empty()) {
      ROS_WARN("task2: Data is not correct.");
      return;
    }
  }
  
  voxel_grid.filter(*image_data_cloud);
  ROS_INFO("Get the callback image: %ld", image_data_cloud->points.size());

  // Test: check in rviz
  sensor_msgs::PointCloud2 colored_cloud_msg;
  colored_cloud_msg.header.frame_id = camera_frame;  
  pcl::toROSMsg(*image_data_cloud, colored_cloud_msg);
  pub2.publish(colored_cloud_msg);

}


///////////////////////////////////////////////////////////////////////////////
bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */
  /* Define pose */
  geometry_msgs::Pose target_pose;
  
  target_pose.orientation = orientation;
  bool success = true;
  bool flag = false;
  
  // move up to get a cloud with all targets
  target_pose.position.x = 0.375;
  target_pose.position.y = 0;
  target_pose.position.z = 0.855;
  success *= moveArm(target_pose);
  if (!success) ROS_WARN("move not success");
  // get pointcloud
  image_data_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ros::Duration(0.2).sleep(); // Sleep for 0.3 second
  sub_flag_task3 = true;
  while (image_data_cloud->empty()) {
    ros::Duration(0.15).sleep(); // Sleep for 0.1 second
    ROS_WARN("please wait for the callback image");
  }
  sub_flag_task3 = false;

  // Definition: target positions
  std::vector<std::vector<geometry_msgs::Point>> all_baskets_positions;
  std::vector<std::vector<geometry_msgs::Point>> all_boxes_positions;

  // Definition: cluster information for particular color and object
  std::vector<pcl::PointIndices> cluster_indices_red_boxes;
  std::vector<pcl::PointIndices> cluster_indices_red_baskets;

  std::vector<pcl::PointIndices> cluster_indices_blue_boxes;
  std::vector<pcl::PointIndices> cluster_indices_blue_baskets;

  std::vector<pcl::PointIndices> cluster_indices_purple_boxes;
  std::vector<pcl::PointIndices> cluster_indices_purple_baskets;
  
  // Definition: target poses
  geometry_msgs::Pose target_pick_pose;
  geometry_msgs::Pose target_drop_pose;
  target_drop_pose.orientation = orientation;
  target_pick_pose.orientation = orientation;
    
  // Definition: pointcloud with certain color
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> red_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> blue_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> purple_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // Definition: RGB conditions for filter
  pcl::PointXYZRGB red_min_color(110, 10, 10);
  pcl::PointXYZRGB red_max_color(255, 90, 90);
  pcl::PointXYZRGB blue_min_color(10, 10, 110);
  pcl::PointXYZRGB blue_max_color(90, 90, 255);
  pcl::PointXYZRGB purple_min_color(110, 10, 110);
  pcl::PointXYZRGB purple_max_color(255, 90, 255);

  
  // filter: downsize the pointcloud
  
  if(image_data_cloud->points.size() < 100) ROS_ERROR("Error: image is filted to none. ");    
  ROS_INFO("++============:image_data_cloud %ld", image_data_cloud->points.size());

  // Extract red points
  ROS_INFO("++==red");
  colorFilter(image_data_cloud, red_cloud, red_min_color, red_max_color, false);
  // Extract blue points
  ROS_INFO("++==blue");
  colorFilter(image_data_cloud, blue_cloud, blue_min_color, blue_max_color, false);
  // Extract blue points
  ROS_INFO("++==purple");
  colorFilter(image_data_cloud, purple_cloud, purple_min_color, purple_max_color, false);

  /* Extract Object Centers */
  // Set the filter
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(image_data_cloud);
  ec.setSearchMethod(tree);      // Set the search method
  ec.setInputCloud(image_data_cloud); 
  // Definition: pointcloud of boxes with a color 
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> red_cloud_box(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> blue_cloud_box(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> purple_cloud_box(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Definition: pointcloud of basket with a color 
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> red_cloud_basket(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> blue_cloud_basket(new pcl::PointCloud<pcl::PointXYZRGB>);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> purple_cloud_basket(new pcl::PointCloud<pcl::PointXYZRGB>);

  
  
  
  
  // Extract boxes' centers and baskets' centers
  getColorCloud(ec, tree, red_cloud, red_cloud_box, red_cloud_basket, cluster_indices_red_baskets, cluster_indices_red_boxes);
  getColorCloud(ec, tree, blue_cloud, blue_cloud_box, blue_cloud_basket, cluster_indices_blue_baskets, cluster_indices_blue_boxes);
  getColorCloud(ec, tree, purple_cloud, purple_cloud_box, purple_cloud_basket, cluster_indices_purple_baskets, cluster_indices_purple_boxes);
  
  // Publish for testing
  sensor_msgs::PointCloud2 colored_cloud_msg_red;
  sensor_msgs::PointCloud2 colored_cloud_msg_blue;
  sensor_msgs::PointCloud2 colored_cloud_msg_purple;

  sensor_msgs::PointCloud2 colored_cloud_msg_red_box;
  sensor_msgs::PointCloud2 colored_cloud_msg_blue_box;
  sensor_msgs::PointCloud2 colored_cloud_msg_purple_box;

  colored_cloud_msg_red.header.frame_id = camera_frame;  
  colored_cloud_msg_blue.header.frame_id = camera_frame;  
  colored_cloud_msg_purple.header.frame_id = camera_frame;  

  colored_cloud_msg_red_box.header.frame_id = camera_frame;  
  colored_cloud_msg_blue_box.header.frame_id = camera_frame;  
  colored_cloud_msg_purple_box.header.frame_id = camera_frame;  
  pcl::toROSMsg(*red_cloud_box, colored_cloud_msg_red);
  pub11.publish(colored_cloud_msg_red);
  pcl::toROSMsg(*blue_cloud_box, colored_cloud_msg_blue);
  pub22.publish(colored_cloud_msg_blue);
  pcl::toROSMsg(*purple_cloud_box, colored_cloud_msg_purple);
  pub33.publish(colored_cloud_msg_purple);
  pcl::toROSMsg(*red_cloud_basket, colored_cloud_msg_red_box);
  pub1.publish(colored_cloud_msg_red_box);
  pcl::toROSMsg(*blue_cloud_basket, colored_cloud_msg_blue_box);
  pub2.publish(colored_cloud_msg_blue_box);
  pcl::toROSMsg(*purple_cloud_basket, colored_cloud_msg_purple_box);
  pub3.publish(colored_cloud_msg_purple_box);

  // move closer to the ground for optimized movement during the task
  if(red_cloud_box->empty() && blue_cloud_box->empty() && purple_cloud_box->empty()) return true;
  // target_pose.position.x = 0.2;
  // target_pose.position.y = 0.125;
  // target_pose.position.z = 0.4;
  // success *= moveArm(target_pose);
  // if (!success) ROS_WARN("move not success");
  /* Get all target's position */
  // get transform matrix parameters
  transformPoint();
  // red boxes and baskets
  if (cluster_indices_red_boxes.size() != 0 && cluster_indices_red_baskets.size() != 0)
  {
    getPositions(red_cloud_box, all_boxes_positions, cluster_indices_red_boxes);
    getPositions(red_cloud_basket, all_baskets_positions, cluster_indices_red_baskets);
    std::cout<<"++++++red"<< cluster_indices_red_boxes.size();

  }
  // blue boxes and baskets
  if (cluster_indices_blue_boxes.size() != 0 && cluster_indices_blue_baskets.size() != 0)
  {
    getPositions(blue_cloud_box, all_boxes_positions, cluster_indices_blue_boxes);
    getPositions(blue_cloud_basket, all_baskets_positions, cluster_indices_blue_baskets);
    std::cout<<"++++++blue"<< cluster_indices_blue_boxes.size();
  }
  // purple boxes and baskets
  if (cluster_indices_purple_boxes.size() != 0 && cluster_indices_purple_baskets.size() != 0)
  {
    getPositions(purple_cloud_box, all_boxes_positions, cluster_indices_purple_boxes);
    getPositions(purple_cloud_basket, all_baskets_positions, cluster_indices_purple_baskets);
    std::cout<<"++++++purple, "<< cluster_indices_purple_boxes.size();
  }

  /* Pick and drop */
  for (int i = 0; i < all_baskets_positions.size(); i++)
  {
    
    target_drop_pose.position.x = all_baskets_positions[i][0].x;
    target_drop_pose.position.y = all_baskets_positions[i][0].y;
    target_drop_pose.position.z = basket_length;

    for (int j = 0; j < all_boxes_positions[i].size(); j++)
    {
      if (std::isnan(all_boxes_positions[i][j].x) || std::isnan(all_boxes_positions[i][j].y) || std::isnan(all_boxes_positions[i][j].z) ||
        std::isinf(all_boxes_positions[i][j].x) || std::isinf(all_boxes_positions[i][j].y) || std::isinf(all_boxes_positions[i][j].z)) {
        // The point is considered empty or uninitialized
         ROS_WARN("INVALID");
        continue;
      }
      target_pick_pose.position.x = all_boxes_positions[i][j].x;
      target_pick_pose.position.y = all_boxes_positions[i][j].y;
      target_pick_pose.position.z = cube_length;
      
      ROS_INFO("target: (%f, %f, %f)", target_pick_pose.position.x, target_pick_pose.position.y, target_pick_pose.position.z);
      ROS_INFO("droptarget: (%f, %f, %f)", target_drop_pose.position.x, target_drop_pose.position.y, target_drop_pose.position.z);
      
      success *= pickObj(target_pick_pose);

      success *= dropObj(target_drop_pose);

      // drop in different baskets with the same color
      if (all_baskets_positions[i].size() >= all_boxes_positions[i].size() && j < all_baskets_positions.size() - 1)
      {
        ROS_WARN("%d, %d, %d", all_baskets_positions[i].size(), all_boxes_positions[i].size(), j);
        target_drop_pose.position.x = all_baskets_positions[i][j].x;
        target_drop_pose.position.y = all_baskets_positions[i][j].y;
      }
    }    
  }
  target_drop_pose.position.z += 0.3;
  success *= moveArm(target_drop_pose);
  return true;
}

////////////////////////////////////////////
/**
 * @brief Move the robot gripper to a specified width.
 * 
 * @param width The desired width of the gripper.
 * @return True if the movement was successful, false otherwise.
 */
bool
cw1::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("moveGripper: Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

/**
 * @brief Move the robot arm to a target pose.
 * 
 * @param target_pose The target pose to move the arm to.
 * @return True if the movement was successful, false otherwise.
 */
bool cw1::moveArm(geometry_msgs::Pose target_pose)
{
  // Step 1: Setup the target pose
  ROS_INFO("moveArm: Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // Step 2: Create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Step 3: Log the success of planning the path
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");
  if (not success) return success;
  // Step 4: Execute the planned path
  arm_group_.move();

  // Return the success status of the movement
  return success;
}

/**
 * @brief This function picks up an object using a pose.
 *        The given point is where the centre of the gripper fingers will converge.
 * 
 * @param pose The pose of the object to be picked up.
 * @return True if the pick operation is successful, false otherwise.
 */
bool cw1::pickObj(geometry_msgs::Pose pose)
{
  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose = pose;
  grasp_pose.position.z += z_offset_obj;
  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_box;

  /* Now perform the pick */
  bool success = true;
  ROS_INFO("Begining pick operation");
  // move the arm above the object
  success = moveArm(approach_pose);
  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    success = moveArm(approach_pose);
  }
  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }
  // approach to grasping pose
  success *= moveArm(grasp_pose);
  while (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    grasp_pose.position.z -= 0.005;
    success = moveArm(grasp_pose);
  }
  // grasp!
  success *= moveGripper(gripper_closed_);
  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }
  // retreat with object
  approach_pose.position.z += z_offset_obj;
  success *= moveArm(approach_pose);
  while (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed: picked up, %f, %f", approach_pose.position.x, approach_pose.position.z);
    approach_pose.position.x -= 0.005;
    approach_pose.position.z += 0.01;

    success = moveArm(approach_pose);
  }
  ROS_INFO("Pick operation successful");
  return success;
}



/**
 * @brief Move the robot arm to drop an object into the basket.
 *
 * This function moves the robot arm to drop an object into the basket. 
 * It first calculates the drop pose by adjusting the given pose with an approach distance.
 * Then, it sets a pre-grasping pose by further adjusting the drop pose with a z-offset.
 * The arm moves to the drop pose, and if successful, the gripper opens to drop the object.
 * If any step fails, an error message is logged, and the function returns false.
 *
 * @param pose The pose of the object to be dropped.
 * @return True if the object was successfully dropped, false otherwise.
 */
bool cw1::dropObj(geometry_msgs::Pose pose)
{
  // move arm: go to the basket
  geometry_msgs::Pose drop_pose;
  drop_pose = pose;
  drop_pose.position.z += approach_distance_basket;
  drop_pose.position.z += cube_length;

  // move arm to drop pose: plus cube_length to avoid the box collide with the basket
  int success = moveArm(drop_pose); 
  while (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move arm to drop pose");
    drop_pose.position.z += 0.05;
    // move arm to drop pose
    success = moveArm(drop_pose); 
  }

  // move gripper: drop
  success *= moveGripper(gripper_open_);

  if (not success) 
  {
    ROS_ERROR("Retreating arm after picking failed. move gripper: drop");
    return false;
  }
  // leave the basket: get higher to avoid collide during movement
  drop_pose.position.z += z_offset_obj;
  success *= moveArm(drop_pose);
  while (not success) 
  {
    drop_pose.position.z += 0.01;
    success = moveArm(drop_pose); 
  } 
  return success;
}



// Function: transformPoint
// Description: Transforms a point from one frame to another using TF2 library.
void cw1::transformPoint() {
    // Declare a variable to hold the transformed point
    geometry_msgs::PointStamped point_out;

    // Initialize a TF2 TransformListener object with the provided buffer
    tf2_ros::TransformListener tfListener(cw1::tf_buffer);
    
    try {
        // Wait for the transform to become available with a timeout of 1 second
        // This ensures that the transform is available before proceeding
        tf_buffer.canTransform(target_frame, camera_frame, ros::Time(0), ros::Duration(1.0));

        // Get the latest transform between the target and camera frames
        transform_matrix = tf_buffer.lookupTransform(target_frame, camera_frame, ros::Time(0));
        
    } catch (tf2::TransformException& ex) {
        // Handle any exceptions that occur during the transform lookup
        ROS_WARN("Failed to transform point: %s", ex.what());
    }
}



void cw1::getPositions(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_point_cloud_filted, 
std::vector<std::vector<geometry_msgs::Point>> &all_positions, std::vector<pcl::PointIndices> &cluster_indices)
{
  /**
 * @brief Extract positions from a filtered point cloud and cluster indices.
 * 
 * This function extracts positions from a filtered point cloud based on provided cluster indices.
 * 
 * @param image_point_cloud_filted The filtered point cloud.
 * @param all_positions A reference to a vector of vectors containing the positions of clusters for different color.
 * @param cluster_indices A reference to a vector of point indices representing clusters in the point cloud.
 * 
 * @note This function assumes that the point cloud has already been filtered.
 * 
 * @note The positions are represented as vectors of geometry_msgs::Point.
 */

  std::vector<geometry_msgs::Point> positions; 
  // Initialize a vector to store the centroid of a cluster.
  Eigen::Vector4f centroid;
  // If there are no cluster indices provided, return early as there is no work to be done.
  if (cluster_indices.empty()) return;

  // Create points in the camera and target frames
  // Initialize point structures to represent points in the camera and target frames.
  geometry_msgs::PointStamped point_in;
  geometry_msgs::PointStamped point_out;

  // Iterate through each set of cluster indices.
  for (const pcl::PointIndices& indices : cluster_indices) 
  {
    // Create a shared pointer to store the point cloud of the current cluster.
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Set the frame IDs for the points.
    point_in.header.frame_id = camera_frame;
    point_out.header.frame_id = target_frame;

    // Add the point at the current index to the cluster.
    for (int index : indices.indices) 
    {
      clusters->points.push_back(image_point_cloud_filted->points[index]);
    }

    // Calculate the centroid of the cluster.
    pcl::compute3DCentroid(*clusters, centroid);
    // Set the coordinates of the centroid point in the camera frame.
    point_in.point.x = centroid[0];
    point_in.point.y = centroid[1];
    point_in.point.z = centroid[2];
    point_in.header.stamp = ros::Time::now();
    // Transform the centroid point to the target frame.
    tf2::doTransform(point_in.point, point_out.point, transform_matrix);
    ROS_INFO("target2: (%f, %f, %f)", point_out.point.x, point_out.point.y, point_out.point.z);
    // Add the transformed centroid point to the vector of positions.
    positions.push_back(point_out.point);

  }
  // Add the vector of positions for all clusters to the main vector of positions.
  all_positions.push_back(positions);
}

void cw1::colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_out,
                 pcl::PointXYZRGB min_color, pcl::PointXYZRGB max_color, bool negative) {
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    /**
   * @brief Apply color filtering to a point cloud.
   * 
   * This function filters the input point cloud based on the specified color range.
   * Points within the range are retained or excluded based on the 'negative' parameter.
   * 
   * @param cloud_in Input point cloud to be filtered.
   * @param cloud_out Output point cloud containing filtered points.
   * @param min_color Minimum color values for filtering.
   * @param max_color Maximum color values for filtering.
   * @param negative Flag indicating whether to retain points within the color range (false) or exclude them (true).
   */

    for (size_t i = 0; i < cloud_in->size(); ++i) {
        pcl::PointXYZRGB point = cloud_in->points[i];
        if ((point.r >= min_color.r && point.r <= max_color.r) &&
            (point.g >= min_color.g && point.g <= max_color.g) &&
            (point.b >= min_color.b && point.b <= max_color.b)) {
            indices->indices.push_back(i);
        }
    }
    // Check if any indices were found for filtering
    if (indices->indices.empty()) {
        ROS_WARN("No points found within the specified color range. Output cloud will be empty.");
        cloud_out->clear();
        return;
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices);
    extract.setNegative(negative);  // Set to true to remove points in the specified range
    extract.filter(*cloud_out);
}


void cw1::getColorCloud(pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>& ec, pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree, pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_basket, std::vector<pcl::PointIndices>& cluster_indices_baskets, std::vector<pcl::PointIndices>& cluster_indices_boxes)
{ 
  /**
   * @brief Extract color clouds for different objects from a clustered point cloud.
   * 
   * This function extracts color clouds corresponding to different objects (e.g., boxes, baskets)
   * from a clustered point cloud using Euclidean clustering.
   * 
   * @param ec EuclideanClusterExtraction object for clustering.
   * @param tree KdTree object for nearest neighbor search.
   * @param color_cloud Input point cloud containing colored points.
   * @param color_cloud_box Output point cloud for storing points belonging to boxes.
   * @param color_cloud_basket Output point cloud for storing points belonging to baskets.
   * @param cluster_indices_baskets Indices of clusters corresponding to baskets.
   * @param cluster_indices_boxes Indices of clusters corresponding to boxes.
   */

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (color_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.744, 0.772);
  pass.setNegative (false);
  pass.filter (*color_cloud_box);
  if(color_cloud_box->points.size() < 250) 
  {
    ROS_WARN("No color_cloud_box. ");
    return;
  }
  // Extract box clusters from the cloud with boxes and part of baskets, and store the cluster indices in cluster_indices_boxes
  // box length is 0.04. set the tolerance near box length (avoid detecting two boxes as one box)
  extractObjectsClusters(color_cloud_box, cluster_indices_boxes, 0.04, 280, 400);


  if(cluster_indices_boxes.size() == 0) 
  {
    ROS_WARN("No boxes. %ld", color_cloud_box->points.size());
    return;
  }  

  // basket upper cloud
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(0, 0.71); // only use upper half cloudpoint of baskets (there is only basket in these area, avoid mix boxes together with baskets)
  pass.filter (*color_cloud_basket);

  if (color_cloud_basket->empty()) {
    ROS_WARN("No basket cloud!, %ld, %ld", color_cloud_basket->size(), color_cloud->size());
    return;
  }
  // Extract baskets clusters from the upper basket cloud and store the cluster indices in cluster_indices_baskets
  // basket length is 0.1. set the tolerance larger than basket_length*sqrt(2) (avoid detecting two baskets as one)
  extractObjectsClusters(color_cloud_basket, cluster_indices_baskets, 0.145, 400, 1000);

  if(cluster_indices_baskets.size() == 0) 
  {
    ROS_WARN("No basket in this color. Cloud size: %ld", color_cloud_basket->points.size());
    return;
  }
  if(cluster_indices_baskets.size() > 1) ROS_WARN("More than one basket in this color. ");
  ROS_INFO("============A cloud, baskets:%ld, boxes:%ld, box size: %d, color_cloud_basket size: %ld", cluster_indices_baskets.size(), cluster_indices_boxes.size(), cluster_indices_boxes[0].indices.size(), color_cloud_basket->size());
}


void cw1::extractObjectsClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, std::vector<pcl::PointIndices>& cluster_indices, 
float tolerance, int min_cluster_size, int max_cluster_size)
{
  // Extract baskets clusters from the cloud and store the cluster indices in cluster_indices
  ec.setInputCloud(color_cloud);
  ec.setClusterTolerance(tolerance);  // Set the tolerance for cluster segmentation
  ec.setMinClusterSize(min_cluster_size);     // Set the minimum number of points in a cluster
  ec.setMaxClusterSize(max_cluster_size);   // Set the maximum number of points in a cluster
  ec.extract(cluster_indices);
}