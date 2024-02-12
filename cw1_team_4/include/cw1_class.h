/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Image.h>
// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// ROS includes
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
// TF specific includes
#include <tf/transform_listener.h>

// #include <thread>
class cw1
{
public: 
  /* ----- class member variables ----- */

  // ROS variables
  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  ros::Subscriber sub_task; // subscirber for pointcloud

  ros::Publisher pub;


  /** \brief  Server for advertising set_arm_srv_  service.  */
  ros::ServiceServer set_arm_srv_;

  /** \brief  Server for advertising set_arm_srv_  service.  */
  ros::ServiceServer set_gripper_srv_;

  /** \brief  Server for advertising pick_srv_  service. */
  ros::ServiceServer pick_srv_;


  /** \brief Define some useful constant values. */  
  cw1(ros::NodeHandle nh);
  float cube_length = 0.04;
  double gripper_open_ = 0.075;
  double gripper_closed_ = 0.02;
  float basket_length = 0.1;
  /** \brief Parameters to define the pick operation */
  double z_offset_obj = 0.125;
  double angle_offset_ = 3.14159 / 4.0;
  // double approach_distance_basket = 0.205;
  double approach_distance_basket = 0.155;
  // double approach_distance_box = 0.165;
  double approach_distance_box = 0.135;

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /** \brief MoveIt interface to move groups to seperate the arm,
    * these are defined in urdf. */
  std::vector<moveit::planning_interface::MoveGroupInterface> arm_groups;

  // Orientation for all tasks
  geometry_msgs::Quaternion orientation;

 /* Task2 */
  ros::Subscriber sub_task2; // subscirber for pointcloud
  std::vector<uint8_t> imageData;
  std::vector<float> BLUE = {0.1, 0.1, 0.8};
  std::vector<float> RED = {0.8, 0.1, 0.1};
  std::vector<float> PURPLE = {0.8, 0.1, 0.8};
  // flag for pointcloud information
  bool sub_flag_task2 = false;

  // Pointcloud for task2 and 3
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> image_data_cloud;


  /* Task3 */
  std::vector<std::string> box_colours;
  geometry_msgs::Point position_camera;

  // tf
  tf2_ros::Buffer tf_buffer;
  std::string target_frame = "panda_link0";
  std::string camera_frame = "color";
  std::string posi_frame = "panda_hand";
  geometry_msgs::TransformStamped transform_matrix;
  geometry_msgs::TransformStamped inv_transform_matrix;
  

  // filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  // cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // flag for pointcloud information
  bool sub_flag_task3 = false;

  // height of arm when taking the camera information
  float arm_height = 0.856; 



public:

  /* ----- class member functions ----- */


  /***************************Callback Functions***************************/

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  // subscruber callback for task 2 and 3
  void imageColorCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /***************************Movement Functions***************************/
  /* ------ move gripper ------*/
  bool moveGripper(float width);
  /* --------- move arm ---------*/
  bool moveArm(geometry_msgs::Pose target_pose);
  /* --------PICK-------- */
  bool pickObj(geometry_msgs::Pose pose);
  /* --------DROP-------- */
  bool dropObj(geometry_msgs::Pose pose);
  

  /***************************TF Function***************************/
  // Function: transformPoint
  // Description: Transforms a point from one frame to another using TF2 library.
  void transformPoint();
  
  
  /***************************Pointcloud Functions***************************/

  /**
   * @brief getPositions: Extract positions from a filtered point cloud and cluster indices.
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
  void getPositions(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_point_cloud_filted, 
std::vector<std::vector<geometry_msgs::Point>> &all_positions, std::vector<pcl::PointIndices> &cluster_indices);



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
  void colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_out,
                 pcl::PointXYZRGB min_color, pcl::PointXYZRGB max_color, bool negative) ;

  /**
   * @brief Extract color clouds for different objects from a clustered point cloud.
   * 
   * This function extracts color clouds corresponding to different objects (e.g., boxes, baskets)
   * from a clustered point cloud using Euclidean clustering.
   * 
   * @param ec EuclideanClusterExtraction object for clustering.
   * @param color_cloud Input point cloud containing colored points.
   * @param color_cloud_box Output point cloud for storing points belonging to boxes.
   * @param color_cloud_basket Output point cloud for storing points belonging to baskets.
   * @param cluster_indices_baskets Indices of clusters corresponding to baskets.
   * @param cluster_indices_boxes Indices of clusters corresponding to boxes.
   */
  void getColorCloud(pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>& ec, pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_box, pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_basket, std::vector<pcl::PointIndices>& cluster_indices_baskets, std::vector<pcl::PointIndices>& cluster_indices_boxes);

  void extractObjectsClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, std::vector<pcl::PointIndices>& cluster_indices, 
float tolerance, int min_cluster_size, int max_cluster_size);
  

};

#endif // end of include guard for CW1_CLASS_H_
