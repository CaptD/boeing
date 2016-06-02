#include <robot_control/robot.h>

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

RobotControl::RobotControl(ros::NodeHandle &nh)
{
    nh.param("freq",        _freq,          100.0);
    nh.param("gravity",     _gravity,       9.8);


    _init_time = true;
    _init_ctrl = true;

    _linear_velocity.setZero();
    _linear_velocity.setX(-0.05);
    _angular_velocity.setZero();

    _pose_sub = nh.subscribe("/gazebo/model_states", 10, &RobotControl::pose_callback, this);

    _model_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    _joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);

    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.name[0] ="trans_base_joint";
    joint_state.name[1] ="Arm_yaw_joint";
    joint_state.name[2] ="arm_pitch_joint";
    joint_state.name[3] ="arm_prismatic_joint";
    joint_state.name[4] ="wrist_pitch_joint";
    joint_state.name[5] ="wrist_roll_joint";
    joint_state.name[6] ="cmass_left_joint";
    joint_state.name[7] ="cmass_right_joint";

}

void RobotControl::pose_callback(const gazebo_msgs::ModelStates::Ptr &msg)
{
    for(int i = 0; i < msg->name.size(); i++) {
        if(msg->name[i].compare("crawler") == 0) {
            _position[0] = msg->pose[i].position.x;
            _position[1] = msg->pose[i].position.y;
            _position[2] = msg->pose[i].position.z;

            update_time();
            update_state();
            publish_state();
            publish_tf();
            publish_joint();
        }
    }

}
void RobotControl::update_time()
{
    _time = ros::Time::now().toSec();

    if(_init_time) {
        _dt = 1.0 / _freq;
        _init_time = false;
    }
    else {
        _dt = _time - _time_old;
    }
    _time_old = _time;

    if(fabs(_dt) > 1.5 * 1.0 / _freq || fabs(_dt) < 0.7 * 1.0 / _freq) {
        _dt = 1.0 / _freq;
    }
}

void RobotControl::update_state()
{
    if(_position.x() < -0.5) {
        _linear_velocity.setX(0.05);
    }
    if(_position.x() > 1.0) {
        _linear_velocity.setX(-0.05);
    }
    _position    = _position    + _linear_velocity  * _dt;
    _orientation = _orientation + _angular_velocity * _dt;

}

void RobotControl::publish_state()
{
    gazebo_msgs::ModelState msg;
    msg.model_name = "crawler";
    msg.pose.position.x = _position.x();
    msg.pose.position.y = _position.y();
    msg.pose.position.z = _position.z();

    tf::Quaternion q;
    //_quaternion.setRPY(_orientation[0], _orientation[1], _orientation[2]);
    _quaternion.setRPY(0.0, 0.0, M_PI);
    msg.pose.orientation.w = _quaternion.w();
    msg.pose.orientation.x = _quaternion.x();
    msg.pose.orientation.y = _quaternion.y();
    msg.pose.orientation.z = _quaternion.z();

    _model_pub.publish(msg);

}

void RobotControl::publish_tf()
{
    ROS_INFO_ONCE("publishing tf.");
    // publish tf
    tf::Transform transform;
    transform.setOrigin(_position);
    transform.setRotation(_quaternion);
    _tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"gazebo_world","chasis_link"));

}

void RobotControl::publish_joint()
{
    ROS_INFO_ONCE("publishing joints.");
    // publish joints
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0; // trans_base_joint
    joint_state.position[1] = M_PI; // Arm_yaw_joint
    joint_state.position[2] = 0; // arm_pitch_joint
    joint_state.position[3] = 0; // arm_prismatic_joint
    joint_state.position[4] = 0; // wrist_pitch_joint
    joint_state.position[5] = 0; // wrist_roll_joint
    joint_state.position[6] = 0; // cmass_left_joint
    joint_state.position[7] = 0; // cmass_right_joint
    _joint_pub.publish(joint_state);

}
