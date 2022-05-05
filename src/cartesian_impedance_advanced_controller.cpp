// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_advanced_controllers/cartesian_impedance_advanced_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka_advanced_controllers/franka_model.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "pseudo_inversion.h"
namespace franka_advanced_controllers {

bool CartesianImpedanceAdvancedController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &CartesianImpedanceAdvancedController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &CartesianImpedanceAdvancedController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  // We want to add the subscriber to the note for reading the desired stiffness in the different directions
  sub_stiffness_ = node_handle.subscribe(
    "/stiffness", 20, &CartesianImpedanceAdvancedController::equilibriumStiffnessCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());
  // sub_equilibrium_pose_ik = node_handle.subscribe(
  //       "/equilibrium_pose", 1, &CartesianImpedanceAdvancedController::equilibriumConfigurationIKCallback, this,
  //       ros::TransportHints().reliable().tcpNoDelay());
  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
    "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);

  pub_cartesian_pose_= node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose",1);

  pub_force_torque_= node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext",1);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceAdvancedController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceAdvancedController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceAdvancedController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceAdvancedController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceAdvancedController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceAdvancedController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceAdvancedController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceAdvancedController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<franka_advanced_controllers::compliance_paramConfig>(
          dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceAdvancedController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  //position_d_target_.setZero();
  //orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  stiff_.setZero();

  return true;
}

void CartesianImpedanceAdvancedController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  // set equilibrium point to current state
  position_d_ = initial_transform.translation(); // this allows the robot to start on the starting configuration
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear()); // this allows the robot to start on the starting configuration
  //position_d_target_ = initial_transform.translation();
  //orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  q_d_= q_initial; //TODO check this
  force_torque_old.setZero();
  double time_old=ros::Time::now().toSec();
  calculateDamping(q_d_);
  ROS_INFO_STREAM("Damping matrix is:" << cartesian_damping_target_);
  calculateDamping_NullSpace(q_d_);
  ROS_INFO_STREAM("Nullspace Damping  matrix is:" << nullspace_damping_target_);
}

void CartesianImpedanceAdvancedController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
  Eigen::MatrixXd M_curr_inv = mass.inverse();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  q_t_ = q;
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  double time_=ros::Time::now().toSec();
  //ddq=ddq+(dq-dq_old)/(time_-time_old);
  //dq_old=dq;
  //time_old=time_;
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
  std::array<double, 7> gravity = model_handle_->getGravity();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix<double, 7, 1>  tau_f;
  Eigen::MatrixXd jacobian_transpose_pinv;
  Eigen::MatrixXd Null_mat;
  Eigen::MatrixXd Null_mat_dyn;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  tau_f(0) =  FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
  tau_f(1) =  FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
  tau_f(2) =  FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
  tau_f(3) =  FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
  tau_f(4) =  FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
  tau_f(5) =  FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
  tau_f(6) =  FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;

  force_torque=force_torque-jacobian_transpose_pinv*(tau_ext-tau_f);

  // publish force, torque
  filter_step=filter_step+1;
  filter_step_=10;
  alpha=1;
  if (filter_step==filter_step_){
    geometry_msgs::WrenchStamped force_torque_msg;
    force_torque_msg.wrench.force.x=force_torque_old[0]*(1-alpha)+force_torque[0]*alpha/(filter_step_);
    force_torque_msg.wrench.force.y=force_torque_old[1]*(1-alpha)+ force_torque[1]*alpha/(filter_step_);
    force_torque_msg.wrench.force.z=force_torque_old[2]*(1-alpha)+force_torque[2]*alpha/(filter_step_);
    force_torque_msg.wrench.torque.x=force_torque_old[3]*(1-alpha)+force_torque[3]*alpha/(filter_step_);
    force_torque_msg.wrench.torque.y=force_torque_old[4]*(1-alpha)+force_torque[4]*alpha/(filter_step_);
    force_torque_msg.wrench.torque.z=force_torque_old[5]*(1-alpha)+force_torque[5]*alpha/(filter_step_);
    pub_force_torque_.publish(force_torque_msg);
    force_torque_old=force_torque/(filter_step_);
    force_torque.setZero();
    //ddq.setZero();
    filter_step=0;
    }

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x=position[0];
  pose_msg.pose.position.y=position[1];
  pose_msg.pose.position.z=position[2];
  pose_msg.pose.orientation.x=orientation.x();
  pose_msg.pose.orientation.y=orientation.y();
  pose_msg.pose.orientation.z=orientation.z();
  pose_msg.pose.orientation.w=orientation.w();
  pub_cartesian_pose_.publish(pose_msg);
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), null_vect(7), tau_joint_limit(7);

  // Calculate D online
  calculateDamping(q_t_); // compute the damping before than send pose to the attractor
  calculateDamping_NullSpace(q_t_); // compute the damping before than send pose to the attractor
  

  //robot_state.q.data()
  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse

  Null_mat=(Eigen::MatrixXd::Identity(7, 7) -jacobian.transpose() * jacobian_transpose_pinv);
  Eigen::MatrixXd M_cart_inv=(jacobian*M_curr_inv*jacobian.transpose());
  Null_mat_dyn= (Eigen::MatrixXd::Identity(7, 7) -jacobian.transpose() *M_cart_inv.inverse()* jacobian* M_curr_inv);
  null_vect.setZero();
  null_vect(0)=(q_d_nullspace_(0) - q(0));
  null_vect(1)=(q_d_nullspace_(1) - q(1));
  null_vect(2)=(q_d_nullspace_(2) - q(2));
  null_vect(3)=(q_d_nullspace_(3) - q(3));
  null_vect(4)=(q_d_nullspace_(4) - q(4));
  null_vect(5)=(q_d_nullspace_(5) - q(5));
  null_vect(6)=(q_d_nullspace_(6) - q(6));
  // Cartesian PD control with damping ratio = 1
  //ROS_INFO_STREAM("Damping matrix is:" << cartesian_damping_);
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error -  cartesian_damping_ * (jacobian * dq)); //double critic damping
  // nullspace PD control with damping ratio = 1
  tau_nullspace << Null_mat *
                       (nullspace_stiffness_ * null_vect -
                        nullspace_damping_ * dq); //critic damping
  tau_joint_limit.setZero();
  for (int i = 0; i <= 6; i++) {
      if (q(i)>q_max[i]-joint_limit_tollerance)     { tau_joint_limit(i)=-10; };
      if (q(i)<q_min[i]+joint_limit_tollerance)    { tau_joint_limit(i)=+10; };
  };
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis+tau_joint_limit+tau_f ;
  // Saturate torque rate to avoid discontinuities
  //tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  //tau_d[6]=std::min(std::max(tau_d[6],-0.05),0.05);
  cartesian_stiffness_ =cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
  nullspace_stiffness_ = nullspace_stiffness_target_;
  nullspace_damping_= nullspace_damping_target_;
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceAdvancedController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  //tau_d_saturated[6]=0.5*std::tanh(tau_d_saturated[6]);
  return tau_d_saturated;
}

void CartesianImpedanceAdvancedController::equilibriumStiffnessCallback(
    const std_msgs::Float32MultiArray::ConstPtr& stiffness_){

  int i = 0;
  // print all the remaining numbers
  for(std::vector<float>::const_iterator it = stiffness_->data.begin(); it != stiffness_->data.end(); ++it)
  {
    stiff_[i] = *it;
    i++;
  }
  for (int i = 0; i < 6; i++){
  for (int j = 0; j < 6; j++) {
  cartesian_stiffness_target_(i,j)=std::max(std::min(stiff_[i+j], float(4000.0)), float(0.0));
  }
    }
  ROS_INFO_STREAM("Stiffness matrix is:" << cartesian_stiffness_target_);
  // calculateDamping(q_d_);
  // calculateDamping_NullSpace(q_d_);
  ROS_INFO_STREAM("Damping matrix is:" << cartesian_damping_target_);
}

void CartesianImpedanceAdvancedController::complianceParamCallback(
    franka_advanced_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0,0)=config.translational_stiffness_X;
  cartesian_stiffness_target_(1,1)=config.translational_stiffness_Y;
  cartesian_stiffness_target_(2,2)=config.translational_stiffness_Z;
  cartesian_stiffness_target_(3,3)=config.rotational_stiffness_X;
  cartesian_stiffness_target_(4,4)=config.rotational_stiffness_Y;
  cartesian_stiffness_target_(5,5)=config.rotational_stiffness_Z;
  damping_ratio_translation=config.damping_ratio_translation;
  damping_ratio_rotation=config.damping_ratio_rotation;
  damping_ratio_nullspace=config.damping_ratio_nullspace;
  for (int i = 0; i < 7; i++){
  nullspace_stiffness_target_(i,i) = config.nullspace_stiffness;}
  // calculateDamping(q_d_);
  // calculateDamping_NullSpace(q_d_);
  ROS_INFO_STREAM("Stiffness matrix is:" << cartesian_stiffness_target_);
  ROS_INFO_STREAM("Damping matrix is:" << cartesian_damping_target_);
  ROS_INFO_STREAM("Nullspace Stiffness matrix is:" << nullspace_stiffness_target_);
  ROS_INFO_STREAM("Nullspace Damping matrix is:" << nullspace_damping_target_);
}



void CartesianImpedanceAdvancedController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  Eigen::Vector3d position_d_ik;
  Eigen::Quaterniond orientation_d_ik;
  position_d_ik << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(orientation_d_);
  orientation_d_ik.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_.coeffs().dot(orientation_d_ik.coeffs()) < 0.0) {
    orientation_d_ik.coeffs() << -orientation_d_ik.coeffs();
}
  geometry_msgs::Pose pose_msg_;
   Eigen::Matrix<double, 7, 1> q_d_damp;
  pose_msg_.position.x=position_d_ik[0];
  pose_msg_.position.y=position_d_ik[1];
  pose_msg_.position.z=position_d_ik[2];
  pose_msg_.orientation.x=orientation_d_ik.coeffs()[0];
  pose_msg_.orientation.y=orientation_d_ik.coeffs()[1];
  pose_msg_.orientation.z=orientation_d_ik.coeffs()[2];
  pose_msg_.orientation.w=orientation_d_ik.coeffs()[3];
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_curr(robot_state.q.data());
  for (int i = 0; i < 7; i++) {
    q_start_ik[i]=q_curr(i);
  }
  KDL::JntArray ik_result = _panda_ik_service.perform_ik(pose_msg_, q_start_ik);
  // KDL::JntArray ik_result = _panda_ik_service.perform_ik(pose_msg_);
  _joints_result = (_panda_ik_service.is_valid) ? ik_result : _joints_result;
  _joints_result.resize(7);
  if (_panda_ik_service.is_valid) {
      for (int i = 0; i < 7; i++)
  {   //_joints_result(i) = _position_joint_handles[i].getPosition();
      q_d_damp(i) = _joints_result(i);
      //_iters[i] = 0;
  }
  // calculateDamping(q_d_damp); // compute the damping before than send pose to the attractor
  // calculateDamping_NullSpace(q_d_damp); // compute the damping before than send pose to the attractor
  ROS_INFO_STREAM("Stiffness matrix of goal is:" << cartesian_stiffness_target_);
  ROS_INFO_STREAM("Damping matrix of goal is:" << cartesian_damping_target_);
  for (int i = 0; i < 7; i++)
  {
      //_joints_result(i) = _position_joint_handles[i].getPosition();
      q_d_(i) = _joints_result(i);
      q_d_nullspace_(i) = _joints_result(i);
      //_iters[i] = 0;
  }
  }
  //Publish the position in the cartesian space after all the matrices and jonit goal have been updated
  // position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  // Eigen::Quaterniond last_orientation_d_(orientation_d_);
  // orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //   msg->pose.orientation.z, msg->pose.orientation.w;
  // if (last_orientation_d_.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
  //   orientation_d_.coeffs() << -orientation_d_.coeffs();
  // }
  position_d_=position_d_ik;
  orientation_d_=orientation_d_ik;
}

void CartesianImpedanceAdvancedController::equilibriumConfigurationCallback( const std_msgs::Float32MultiArray::ConstPtr& joint) {
  int i = 0;
  for(std::vector<float>::const_iterator it = joint->data.begin(); it != joint->data.end(); ++it)
  {
    q_d_nullspace_[i] = *it;
    i++;
  }
  return;
}

//This callback computes the nullspace configuration according to the current position of the robot and the cartesian goal. It depdent on the message that is give to the subscriber
// void CartesianImpedanceAdvancedController::equilibriumConfigurationIKCallback( const geometry_msgs::PoseStampedConstPtr& msg) {
//   geometry_msgs::Pose pose_msg_;
//   position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   Eigen::Quaterniond last_orientation_d_(orientation_d_);
//   orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//       msg->pose.orientation.z, msg->pose.orientation.w;
//   if (last_orientation_d_.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
//     orientation_d_.coeffs() << -orientation_d_.coeffs();s
// }
//   pose_msg_.position.x=position_d_[0];
//   pose_msg_.position.y=position_d_[1];
//   pose_msg_.position.z=position_d_[2];
//   pose_msg_.orientation.x=orientation_d_.coeffs()[0];
//   pose_msg_.orientation.y=orientation_d_.coeffs()[1];
//   pose_msg_.orientation.z=orientation_d_.coeffs()[2];
//   pose_msg_.orientation.w=orientation_d_.coeffs()[3];
//   // use tracik to get joint positions from target pose
//   //std::cout << pose_msg_;
//   franka::RobotState robot_state = state_handle_->getRobotState();
//   Eigen::Map<Eigen::Matrix<double, 7, 1> > q_curr(robot_state.q.data());
//   for (int i = 0; i < 7; i++) {
//     q_start_ik[i]=q_curr(i);
//   }
//   KDL::JntArray ik_result = _panda_ik_service.perform_ik(pose_msg_, q_start_ik);
//   // KDL::JntArray ik_result = _panda_ik_service.perform_ik(pose_msg_);
//   _joints_result = (_panda_ik_service.is_valid) ? ik_result : _joints_result;
//   _joints_result.resize(7);
//   if (_panda_ik_service.is_valid) {
//   for (int i = 0; i < 7; i++)
//   {
//       //_joints_result(i) = _position_joint_handles[i].getPosition();
//       q_d_nullspace_(i) = _joints_result(i);
//       //_iters[i] = 0;
//   }
//   //std::cout << q_d_nullspace_;
//   //return;
// }
// }

void CartesianImpedanceAdvancedController::calculateDamping(Eigen::Matrix<double, 7, 1>& goal_ ){
  for (int i = 0; i < 7; i++) {
  goal[i]=goal_(i);
}
  Eigen::Matrix<double, 6, 6> K_;
  Eigen::Matrix<double, 6, 6> D_;
  //--- mass at goal ---//
  mass_goal_ = model_handle_->getMass(goal, total_inertia_, total_mass_, F_x_Ctotal_);
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass_goal(mass_goal_.data());
  //--- mass at current state ---//
  // std::array<double, 49> mass_array = model_handle_->getMass();
  // Eigen::Map<Eigen::Matrix<double, 7, 7> > mass_goal(mass_goal_.data());

  Eigen::MatrixXd M_inv = mass_goal.inverse();
  // ROS_INFO_STREAM("M_inv:" << M_inv);
  //Compute the Jacobian of the goal
  //Compute inv(J*M_inv*J_T)
  std::array<double, 42> jacobian_array =model_handle_-> getZeroJacobian(franka::Frame::kEndEffector, goal, F_T_EE, EE_T_K);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > J(jacobian_array.data());
  // ROS_INFO_STREAM("J:" << J);
  Eigen::MatrixXd M_cart_inv=(J*M_inv*J.transpose());
  Eigen::MatrixXd phi_mk = M_cart_inv.llt().matrixU();
  //  ROS_INFO_STREAM("phi_mk:" << phi_mk);
  //K_.setIdentity();
  K_=cartesian_stiffness_target_;
  Eigen::MatrixXd sigma_mk_hold = phi_mk * K_ * phi_mk.transpose();
  Eigen::Matrix<double, 6, 1> sigma_mk;
  // ROS_INFO_STREAM("sigma_mk:" << sigma_mk);
  Eigen::MatrixXd U, V;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(sigma_mk_hold, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  //ROS_INFO_STREAM("U:" << U);
  //ROS_INFO_STREAM("V:" << V);
  sigma_mk = svd.singularValues();
  // ROS_INFO_STREAM("sigma_mk:" << sigma_mk);

  Eigen::MatrixXd W = phi_mk.transpose() * U;

  Eigen::Matrix<double, 6, 1> xi_n;
  Eigen::Matrix<double, 6, 1> omega_n;
  Eigen::Matrix<double, 6, 1> sigma_dn;

  for(int k=0;k<3;k++){
    xi_n(k, 0) = damping_ratio_translation;
    omega_n(k, 0) = sqrt(sigma_mk(k, 0));
    sigma_dn(k, 0) = 2.0 * xi_n(k, 0) * omega_n(k, 0);
  }
  for(int k=3;k<6;k++){
  xi_n(k, 0) = damping_ratio_rotation;
  omega_n(k, 0) = sqrt(sigma_mk(k, 0));
  sigma_dn(k, 0) = 2.0 * xi_n(k, 0) * omega_n(k, 0);
  }
  Eigen::MatrixXd Sigma_d = sigma_dn.asDiagonal();

  // ROS_INFO_STREAM("xi_n:" << xi_n);

  Eigen::Matrix<double, 6, 6> W_T = W.transpose();
  D_ = W_T.inverse() * Sigma_d * W.inverse();
  cartesian_damping_target_=D_;
  // ROS_INFO_STREAM("D:" << D_);
}

void CartesianImpedanceAdvancedController::calculateDamping_NullSpace(Eigen::Matrix<double, 7, 1>& goal_ ){
  for (int i = 0; i < 7; i++) {
  goal[i]=goal_(i);
}
  mass_goal_ = model_handle_->getMass(goal, total_inertia_, total_mass_, F_x_Ctotal_);
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass_goal(mass_goal_.data());
  Eigen::Matrix<double, 7, 7> K_;
  Eigen::Matrix<double, 7, 7> D_;
  // ROS_INFO_STREAM("M_inv:" << M_inv);
  //Compute the Jacobian of the goal
  //Compute inv(J*M_inv*J_T)
  std::array<double, 42> jacobian_array =model_handle_-> getZeroJacobian (franka::Frame::kEndEffector, goal, F_T_EE, EE_T_K);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > J(jacobian_array.data());
  Eigen::Matrix<double, 7, 7> N;
  Eigen::MatrixXd J_transpose_pinv;
  pseudoInverse(J.transpose(), J_transpose_pinv);
  N=(Eigen::MatrixXd::Identity(7, 7) -J.transpose() * J_transpose_pinv);
  // ROS_INFO_STREAM("J:" << J);
  Eigen::Matrix<double, 7, 7> M_goal_nullspace;
  M_goal_nullspace = mass_goal ;
  Eigen::Matrix<double, 7, 7> Stiffness_nullspace;
  Stiffness_nullspace=nullspace_stiffness_target_;
  Eigen::MatrixXd M_inv = M_goal_nullspace.inverse();
  Eigen::MatrixXd phi_mk = M_inv.llt().matrixU();
  //  ROS_INFO_STREAM("phi_mk:" << phi_mk);
  K_.setIdentity();
  K_=Stiffness_nullspace;
  Eigen::MatrixXd sigma_mk_hold = phi_mk * K_ * phi_mk.transpose();
  Eigen::Matrix<double, 7, 1> sigma_mk;
  // ROS_INFO_STREAM("sigma_mk:" << sigma_mk);
  Eigen::MatrixXd U, V;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(sigma_mk_hold, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  V = svd.matrixV();
  //ROS_INFO_STREAM("U:" << U);
  //ROS_INFO_STREAM("V:" << V);
  sigma_mk = svd.singularValues();
  // ROS_INFO_STREAM("sigma_mk:" << sigma_mk);

  Eigen::MatrixXd W = phi_mk.transpose() * U;

  Eigen::Matrix<double, 7, 1> xi_n;
  Eigen::Matrix<double, 7, 1> omega_n;
  Eigen::Matrix<double, 7, 1> sigma_dn;

  for(int k=0;k<7;k++){
    xi_n(k, 0) = damping_ratio_nullspace;
    omega_n(k, 0) = sqrt(sigma_mk(k, 0));
    sigma_dn(k, 0) = 2.0 * xi_n(k, 0) * omega_n(k, 0);
  }
  Eigen::MatrixXd Sigma_d = sigma_dn.asDiagonal();

  Eigen::Matrix<double, 7, 7> W_T = W.transpose();
  D_ = W_T.inverse() * Sigma_d * W.inverse();
  nullspace_damping_target_=D_;
  // ROS_INFO_STREAM("cartesian_damping_:" << cartesian_damping_);
}

}  // namespace franka_advanced_controllers

PLUGINLIB_EXPORT_CLASS(franka_advanced_controllers::CartesianImpedanceAdvancedController,
                       controller_interface::ControllerBase)
