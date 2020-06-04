#ifndef MUJOCO_INTERFACE_H
#define MUJOCO_INTERFACE_H

#include "state_manager.h"
#include "mujoco_ros_msgs/SimStatus.h"

class MujocoInterface : public StateManager
{
public:
  MujocoInterface(DataContainer &dc_global);
  virtual ~MujocoInterface() {}

  //update state of Robot from mujoco
  virtual void updateState() override;

  //Send command to Mujoco
  //virtual void sendCommand(Eigen::VectorQd command) override;
  virtual void sendCommand(Eigen::VectorQd command, double sim_time, int control_mode = Torquemode) override;

  //connect to Mujoco_ros
  virtual void connect() override;

  //Toggle play
  void playMujoco();

private:
  DataContainer &dc;

  //void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
  //void sensorStateCallback(const mujoco_ros_msgs::SensorStateConstPtr &msg);
  void simCommandCallback(const std_msgs::StringConstPtr &msg);
  void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
  void simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg);

  ros::Publisher mujoco_joint_set_pub_;
  ros::Publisher mujoco_sim_command_pub_;

  ros::Subscriber mujoco_joint_state_sub_;
  ros::Subscriber mujoco_sensor_state_sub_;
  ros::Subscriber mujoco_sim_command_sub_;
  ros::Subscriber mujoco_sim_time_sub_;
  ros::Subscriber mujoco_sim_status_sub_;

  mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;

  bool sim_runnung;

  bool mujoco_ready = false;
  bool mujoco_init_receive = false;
  bool mujoco_reset = false;

  //bool virtual_joint_from_simlulation = true;

  float mujoco_sim_time;
  float mujoco_sim_last_time;

  bool new_state_trigger = false;

  std::string joint_name_mj[MODEL_DOF];
  //ros::Rate rate_;
  int dyn_hz;
};

#endif