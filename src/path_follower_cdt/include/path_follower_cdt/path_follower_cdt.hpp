#include <iostream>
#include <fstream>      // std::ifstream
#include <stdio.h>
#include <deque>

#include <Eigen/Dense>
#include <Eigen/StdVector>


//// Need to add this in 

void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}
////

enum FOLLOWER_OUTPUT {SEND_NOTHING=0, SEND_STOP_WALKING=1, SEND_COMMAND=2};

class PathFollower{
  public:
    PathFollower();
    
    ~PathFollower(){
    }    

    void setGoalAndEnable(Eigen::Isometry3d new_goal){
      ////
      // Use the new goal to update the desire roll, pitch, yaw
      Eigen::Quaterniond q(new_goal.rotation());
      quat_to_euler(q, des_roll_, des_pitch_, des_yaw_);

      current_goal_ = new_goal;        
      //// TODO: enable with logic - if you want!
    }

    FOLLOWER_OUTPUT computeControlCommand(Eigen::Isometry3d current_pose, int64_t utime );

    void getOutputVelocity(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity){
      output_linear_velocity = output_linear_velocity_;
      output_angular_velocity = output_angular_velocity_;
    }

    Eigen::Isometry3d getCurrentGoal(){
      return current_goal_;
    }

  private:

    // variables:
    Eigen::Isometry3d current_goal_;
    Eigen::Vector3d output_linear_velocity_;
    Eigen::Vector3d output_angular_velocity_;
    
    ////
    
    // initialise the desired variables
    double des_yaw_ = 0;
    double des_pitch_ = 0;
    double des_roll_ = 0;
    //double des_pitch_ = 0;
    //double des_roll_ = 0;
    ////

};
