/*
 * NavigationDemo.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#include "grid_map_cdt/Challenge.hpp"
#include <tf_conversions/tf_eigen.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen_conversions/eigen_msg.h>

using namespace grid_map;
using namespace std::chrono;


namespace grid_map_demos {

NavigationDemo::NavigationDemo(ros::NodeHandle& nodeHandle, bool& success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap"),
      demoMode_(false)
{
  if (!readParameters()) {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &NavigationDemo::callback, this);
  listener_ = new tf::TransformListener();

  outputGridmapPub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/filtered_map", 1, true);
  footstepPlanRequestPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/footstep_plan_request", 10);


  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }

  
  success = true;


  verbose_ = false;
  verboseTimer_ = true;
  plannerEnabled_ = true; // start enabled
}

NavigationDemo::~NavigationDemo()
{
}

bool NavigationDemo::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_)) {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  
  nodeHandle_.param("demo_mode", demoMode_, true);
  if (demoMode_)
    ROS_INFO("In demo mode [%d]. will use a hard coded gridmap bag and robot pose", int(demoMode_) );
  else
    ROS_INFO("In live mode [%d]. will listen for poses continuously", int(demoMode_) );

  return true;
}


void NavigationDemo::tic(){
  lastTime_ = high_resolution_clock::now();
}

std::chrono::duration<double> NavigationDemo::toc(){
  auto nowTime = high_resolution_clock::now();
  duration<double> elapsedTime = duration_cast<milliseconds>(nowTime - lastTime_);
  lastTime_ = nowTime;
  // std::cout << elapsedTime.count() << "ms elapsed" << std::endl;    
  return elapsedTime;
}

void NavigationDemo::callback(const grid_map_msgs::GridMap& message)
{
  if (!plannerEnabled_){
    std::cout << "planner enabled. at the goal? grab a beer!\n";
    return;
  }

  // The all important position goal - get the robot there
  Position pos_goal(8.5,4.0);


  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  if(demoMode_){ // demoMode

    Eigen::Vector3d robot_xyz = Eigen::Vector3d(0.0,0.0,0); //rpy
    Eigen::Vector3d robot_rpy = Eigen::Vector3d(0,0,0); //rpy

    pose_robot.setIdentity();
    pose_robot.translation() = robot_xyz;
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(robot_rpy(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(robot_rpy(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(robot_rpy(0), Eigen::Vector3d::UnitX()); // order is ypr

    pose_robot.rotate( motion_R );

  }else{ // online

    tf::StampedTransform transform;
    try {
        listener_->waitForTransform("/odom", "/base", ros::Time(0), ros::Duration(10.0) );
        listener_->lookupTransform("/odom", "/base", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::transformTFToEigen (transform, pose_robot);
    if (verbose_) std::cout << pose_robot.translation().transpose() << " current pose_robot\n";
  }


  Eigen::Isometry3d pose_chosen_carrot = Eigen::Isometry3d::Identity();
  bool sendCommand = planCarrot(message, pose_robot, pos_goal, pose_chosen_carrot);

  if(sendCommand){
    // Send the carrot to the path follower
    geometry_msgs::PoseStamped m;
    m.header = message.info.header;
    tf::poseEigenToMsg (pose_chosen_carrot, m.pose);
    footstepPlanRequestPub_.publish(m);
  }

}

double NavigationDemo::lookUpValue(double x, double y, grid_map::GridMap& outputMap)
{
  double z = 0.0;
  double value;

  Eigen::Vector3d pt_original(x, y, z);
  Position pt = Position(  pt_original.head(2) );

  if ( outputMap.isInside(pt) ){
    Index pt_index;
    outputMap.getIndex( pt, pt_index );
    Position pt_cell;
    outputMap.getPosition(pt_index, pt_cell);
    return value = outputMap.at("carrot_layer", pt_index);
  }
  else {
    return value = 0.0;
  }

}

double NavigationDemo::lookUpValueIndex(Index pt_index, grid_map::GridMap& outputMap)
{
  double value;

  return value = outputMap.at("traversability", pt_index);

}
bool NavigationDemo::planCarrot(const grid_map_msgs::GridMap& message,
  Eigen::Isometry3d pose_robot, Position pos_goal,
  Eigen::Isometry3d& pose_chosen_carrot)
{
  std::cout << "start - carrot planner\n";
  tic();

  // Compute distance to the goal:
  Position pos_robot( pose_robot.translation().head(2) );
  double current_dist_to_goal = (pos_goal - pos_robot).norm();
  std::cout << "current distance to goal: " << current_dist_to_goal << std::endl;

  // If within 1.5m of goal - stop walking
  if (current_dist_to_goal < 1.5){
    // Determine a carrot pose: x and y from the above. z is the robot's height.
    // yaw in the direction of the carrot. roll,pitch zero
    Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(pos_goal(0), pos_goal(1), 0, 1) ;
    double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));
    if (verbose_) std::cout << carrot_relative_pose.transpose() << " - relative carrot\n";
    if (verbose_) std::cout << carrot_relative_theta << " - relative carrot - theta\n";

    Eigen::Isometry3d pose_chosen_carrot_relative = Eigen::Isometry3d::Identity();
    pose_chosen_carrot_relative.translation() = Eigen::Vector3d( carrot_relative_pose(0),carrot_relative_pose(1),0);
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

    pose_chosen_carrot_relative.rotate( motion_R );
    pose_chosen_carrot = pose_robot * pose_chosen_carrot_relative;
    std::cout << current_dist_to_goal << "m to goal. carrot is goal\n";
    // disable carrot planner
    plannerEnabled_ = false;
    return true;
  }



  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);
  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return false;
  }
  if (verboseTimer_) std::cout << toc().count() << "ms: filter chain\n";


  ////// Put your code here ////////////////////////////////////
  //add a new layer full of zeros
  //outputMap.add("carrots", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  //outputMap.add("carrot_layer", outputMap.get("traversability") );

  //copy data to openCV and do an open cv operation e.g. dilation, blurring, erosion. Then copy back to GridMap
  cv::Mat originalImage, dilatedImage;
  GridMapCvConverter::toImage<unsigned short, 1>(outputMap, "traversability", CV_16UC1, 0.0, 1, originalImage);
  cv::imwrite( "originalImage.png", originalImage );
  cv::Mat strEl = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
  cv::erode(originalImage, dilatedImage, strEl);
  // add your OpenCV operation here
  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(dilatedImage, "carrot_layer", outputMap, 0.0, 1);

  /*
  outputMap.add("carrot_layer", outputMap.get("traversability") );
  grid_map::Matrix& dist_data = outputMap["carrot_layer"];
  int numUpd=0;
  int numNaN=0;
  Index new_index;
  for (GridMapIterator iterator(outputMap); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    double minValue=1.0;
    for (int i=-1; i<=1; i++) {
      new_index(0) = index(0) + i;
      for (int j=-1; j<=1; j++) {
        if (j < 1 && j > -1){
          if (i < 1 && i > -1){
            continue;
          }
        }
        new_index(1) = index(1) + j;
        double value = lookUpValueIndex(new_index, outputMap);
        if (value < 2) {
          //std::cout << value << " Value of grid is:\n";
        }
        if (value < minValue) {
          minValue = value;
          numUpd++;
          //std::cout << " UPDATE!\n";
        }
      }
    }
    //outputMap.at("carrot_layer", index) = maxValue;
    std::cout << minValue << " Minimum value\n";
    dist_data(index(0), index(1)) = minValue;
  }
  std::cout << numUpd << " numUpd:\n";
  */






  double threshold = 0.5;
  double optimal_distance = 50;
  double best_x;
  double best_y; 
  Position potential_pose = pos_robot;
  for (int i=-1; i <= 1; i++) {
    potential_pose[0] = pos_robot[0] + i;
    for (int j=-1; j <= 1; j++){
      if (j < 1 && j > -1) {
        if (i < 1 && i > -1) {
          continue;
        }
      }
      potential_pose[1] = pos_robot[1] + j;
/*
      double maxValue=0.0;
      for (int k=-10; k<=10; k++) {
        for (int m=-10; m<=10; m++) {
          double value = lookUpValue(potential_pose[0]+k/100, potential_pose[1]+m/100, outputMap);
          if (value > maxValue) {
            maxValue = value;
          }
        }
      }
*/
      if(lookUpValue(potential_pose[0], potential_pose[1], outputMap)>threshold){
      //if(maxValue<threshold){
        std::cout << lookUpValue(potential_pose[0], potential_pose[1], outputMap) << "lookupvalue\n";
	if((potential_pose - pos_goal).norm() < optimal_distance){
          best_x = potential_pose[0];
          best_y = potential_pose[1];
          optimal_distance = (potential_pose - pos_goal).norm();
        }
      }
    }
  }
        std::cout << best_x << "x location\n";
        std::cout << best_y << "y location\n";
  
  Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(best_x, best_y, 0, 1) ;
  double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));

  Eigen::Isometry3d pose_chosen_carrot_relative = Eigen::Isometry3d::Identity();
  pose_chosen_carrot_relative.translation() = Eigen::Vector3d( carrot_relative_pose(0),carrot_relative_pose(1),0);
  Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

  pose_chosen_carrot_relative.rotate( motion_R );
  pose_chosen_carrot = pose_robot * pose_chosen_carrot_relative;







  //String[] layers = ["traversability", "slope", "roughness" ];
 /*
  double minvalue=10000;
  double maxvalue=-10000;
  for (GridMapIterator iterator(outputMap); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    double value = lookUpValue(index(0), index(1), outputMap);
    if(value<minvalue){
      minvalue=value;
    }
    if(value>maxvalue){
      maxvalue=value;
    }
  }
  std::cout << minvalue << "minimum\n";
  std::cout << maxvalue << "maximum\n";
 */



  ////// Put your code here ////////////////////////////////////


  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  outputGridmapPub_.publish(outputMessage);
  if (verbose_) std::cout << "finished processing\n";
  if (verboseTimer_) std::cout << toc().count() << "ms: publish output\n";

  std::cout << "finish - carrot planner\n\n";
  
 
  /*
  // REMOVE THIS WHEN YOUR ARE DEVELOPING ----------------
  // create a fake carrot - replace with a good carrot
  std::cout << "test - REPLACE FAKE CARROT!\n";
  pose_chosen_carrot.translation() = Eigen::Vector3d(1.0,0,0);
  // REMOVE THIS -----------------------------------------
  */
  

  return true;
}

} /* namespace */
