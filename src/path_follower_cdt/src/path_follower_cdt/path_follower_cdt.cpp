#include <path_follower_cdt/path_follower_cdt.hpp>

PathFollower::PathFollower(){

  std::cout << "Finished setting up PathFollower\n";

}

double max_angular_velocity_ = 1; 


// constrain angle to be -180:180 in radians
double constrainAngle(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}

// clip commanded angular velocity
void clipAngularVelocity(double &angular_velocity){
  if (angular_velocity > max_angular_velocity_)
    angular_velocity = max_angular_velocity_;
  else if (angular_velocity < -max_angular_velocity_)
    angular_velocity = -max_angular_velocity_;
}

FOLLOWER_OUTPUT PathFollower::computeControlCommand(Eigen::Isometry3d current_pose, int64_t current_utime){

  //double des_yaw_ = 0;
  //double des_pitch_ = 0;
  //double des_roll_ = 0;


  double linear_forward_x = 0;
  double linear_forward_y = 0;
  double roll_velocity_ = 0;
  double pitch_velocity_ = 0;
  double yaw_velocity_ = 0;
  //double angular_velocity_ = 0;
  //// Develop your controller here within the calls
  
  Eigen::Quaterniond q(current_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);

  Eigen::Quaterniond q_goal(current_goal_.rotation());
  quat_to_euler(q_goal, des_roll_, des_pitch_, des_yaw_);


  //Eigen::Vector3d current_angs(current_roll, current_pitch, current_yaw);
  //Eigen::Vector3d des_angs(des_roll_, des_pitch_, des_yaw_);
  //Eigen::Vector3d angErrorRaw = current_angs - des_angs;


  //// Compute the position velocities
  Eigen::Vector3d goalDirection = current_goal_.translation() - current_pose.translation();
  double dist_to_goal = goalDirection.norm(); 
  bool xdist_isNeg = goalDirection(0) < 0;
  std::cout << "Current yaw:  " << current_yaw  << "\n";
  std::cout << "Desired yaw:  " << des_yaw_  << "\n";
  
  double rollErrorRaw = current_roll - des_roll_;
  double rollError = constrainAngle(rollErrorRaw);
  double pitchErrorRaw = current_pitch - des_pitch_;
  double pitchError = constrainAngle(pitchErrorRaw);
  double yawErrorRaw = current_yaw - des_yaw_;
  double yawError = constrainAngle(yawErrorRaw);
  double angular_gain_p_;
  double k1 = 1;
  double k2 = 1;
  double k3 = 1;
  int sign = 1;
  
  if (xdist_isNeg){
	sign = -1;
  }else { 
        sign = 1;
  }


  if (dist_to_goal >= 1){
    linear_forward_x = sign * (k1 * dist_to_goal* dist_to_goal + k2 * dist_to_goal + k3 )/(dist_to_goal*dist_to_goal);
    angular_gain_p_ = 0;
  }else if (0.1 <= dist_to_goal < 1){
    //angular_gain_p_ = 0.1/(dist_to_goal + yawError);
    angular_gain_p_ = 0;
    linear_forward_x = sign * k1 * dist_to_goal;
  }else if (dist_to_goal < 0.1)
    linear_forward_x = 0; 
    linear_forward_y = 0;
    angular_gain_p_ = 0.4;

  //roll_velocity_ = -rollError * angular_gain_p_;
  //pitch_velocity_ = -pitchError * angular_gain_p_;
  //yaw_velocity_ = 1;
  roll_velocity_ = 0;
  pitch_velocity_ = 0;
  yaw_velocity_ = -yawError * angular_gain_p_;

  clipAngularVelocity(roll_velocity_);
  clipAngularVelocity(pitch_velocity_);  
  clipAngularVelocity(yaw_velocity_);    
  ////

  // set output
  output_linear_velocity_ = Eigen::Vector3d(linear_forward_x, linear_forward_y, 0);
  output_angular_velocity_ = Eigen::Vector3d(roll_velocity_,pitch_velocity_, yaw_velocity_) ;
  
  //output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity_) ;
  
  PathFollower::getOutputVelocity(output_linear_velocity_, output_angular_velocity_);

  return SEND_COMMAND; 
}
