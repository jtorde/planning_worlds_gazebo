// Instructions: launch gazebo, and then run this file. An image of the the random forest created can be seen in the
// paper "Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments [ICRA 2019]

// Author: Jesus Tordesillas Torres

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <Eigen/Dense>

#include <bits/stdc++.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>

#include "gazebo_cube_spawner.h"

#include <sstream>

using gazebo_test_tools::GazeboCubeSpawner;
using ros::NodeHandle;

#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"

GazeboCubeSpawner::GazeboCubeSpawner(NodeHandle& n) : nh(n)
{
  spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}

void GazeboCubeSpawner::spawnCube(const std::string& name, const std::string& frame_id, float x, float y, float z,
                                  float qx, float qy, float qz, float qw, float width, float height, float depth,
                                  float mass)
{
  spawnPrimitive(name, false, frame_id, x, y, z, qx, qy, qz, qw, width, height, depth, mass);
}

void GazeboCubeSpawner::spawnPrimitive(const std::string& name, const bool doCube, const std::string& frame_id, float x,
                                       float y, float z, float qx, float qy, float qz, float qw, float widthOrRadius,
                                       float height, float depth, float _mass)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;

  gazebo_msgs::SpawnModel spawn;
  spawn.request.model_name = name;

  // just so the variable names are shorter..
  float w = widthOrRadius;
  float h = height;
  float d = depth;

  std::stringstream _s;
  if (doCube)
  {
    _s << "<box>\
            <size>"
       << w << " " << h << " " << d << "</size>\
          </box>";
  }
  else
  {
    _s << "<cylinder>\
                <length>"
       << h << "</length>\
                <radius>" << w << "</radius>\
            </cylinder>";
  }
  std::string geometryString = _s.str();

  float mass = _mass;
  float mass12 = mass / 12.0;

  double mu1 = 500;  // 500 for PR2 finger tip. In first experiment had it on 1000000
  double mu2 = mu1;
  double kp = 10000000;  // 10000000 for PR2 finger tip
  double kd = 1;         // 100 for rubber? 1 fir OR2 finger tip

  bool do_surface = false;
  bool do_inertia = true;

  std::stringstream s;
  s << "<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"
    << name << "'>\
        <static>false</static>\
        <link name='link'>";

  // inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  if (do_inertia)
  {
    double xx, yy, zz;
    if (doCube)
    {
      xx = mass12 * (h * h + d * d);
      yy = mass12 * (w * w + d * d);
      zz = mass12 * (w * w + h * h);
    }
    else
    {
      xx = mass12 * (3 * w * w + h * h);
      yy = mass12 * (3 * w * w + h * h);
      zz = 0.5 * mass * w * w;
    }
    s << "<inertial>\
        <mass>"
      << mass << "</mass>\
        <inertia>\
          <ixx>" << xx << "</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>" << yy << "</iyy>\
          <iyz>0.0</iyz>\
          <izz>" << zz << "</izz>\
        </inertia>\
          </inertial>";
  }
  s << "<collision name='collision'>\
        <geometry>"
    << geometryString;
  s << "</geometry>";

  s << "<surface>\
            <contact>\
              <collide_without_contact>true</collide_without_contact>\
            </contact>\
          </surface>";

  if (do_surface)
    s << "<surface>\
            <friction>\
              <ode>\
            <mu>"
      << mu1 << "</mu>\
            <mu2>" << mu2 << "</mu2>\
            <fdir1>0.000000 0.000000 0.000000</fdir1>\
            <slip1>0.000000</slip1>\
            <slip2>0.000000</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0.000000</restitution_coefficient>\
              <threshold>100000.000000</threshold>\
            </bounce>\
            <contact>\
              <ode>\
            <soft_cfm>0.000000</soft_cfm>\
            <soft_erp>0.200000</soft_erp>\
            <kp>" << kp << "</kp>\
            <kd>" << kd << "</kd>\
            <max_vel>100.000000</max_vel>\
            <min_depth>0.001000</min_depth>\
              </ode>\
            </contact>\
        </surface>";
  s << "</collision>\
          <visual name='visual'>";
  s << "<geometry>" << geometryString;
  s << "</geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>Gazebo/Blue</name>\
            </script>\
        </material>\
          </visual>\
        <gravity>0</gravity>\
        </link>\
      </model>\
    </sdf>";

  spawn.request.model_xml = s.str();
  spawn.request.robot_namespace = "cube_spawner";
  spawn.request.initial_pose = pose;
  spawn.request.reference_frame = frame_id;

  // ROS_INFO("Resulting model: \n %s",s.str().c_str());

  // ROS_INFO("Waiting for service");
  spawn_object.waitForExistence();
  // ROS_INFO("Calling service");

  // std::cout<<spawn.request<<std::endl;

  if (!spawn_object.call(spawn))
  {
    ROS_ERROR("Failed to call service %s", SPAWN_OBJECT_TOPIC);
  }
  ROS_INFO("Result: %s, code %u", spawn.response.status_message.c_str(), spawn.response.success);
}

double randMToN(double m, double n)
{
  return m + (rand() / (RAND_MAX / (n - m)));
}

/*void generateCustomWorld(const Eigen::Vector3d& size, double density)
{

}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxblox");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // voxblox::TsdfServer node(nh, nh_private);

  GazeboCubeSpawner spawner(nh);

  int seed;
  double density;

  ros::param::param<int>("~seed", seed, 0);
  ros::param::param<double>("~density", density, 0.1);

  printf("SEED=%d\n", seed);
  printf("DENSITY=%f\n", density);

  // Seed is from 0 - 10 for this benchmark.
  std::srand(seed);
  Eigen::Vector3d size(50.0, 50.0, 5.0);  // world size

  // Free space around the edges (so we know we don't start in collision).
  Eigen::Vector3d free_space_bounds(2.0, 2.0, 2.0);

  // Some mins and maxes... All objects gotta be on the floor. Just because.
  const double kMinHeight = 2.0;
  const double kMaxHeight = 5.0;
  const double kMinRadius = 0.25;
  const double kMaxRadius = 1.0;

  double usable_area = size.x() * size.y();
  int num_objects = static_cast<int>(std::floor(density * usable_area));

  double total_volume = 0;
  for (int i = 0; i < num_objects; ++i)
  {
    // First select size; pose depends on size in z.

    double height = randMToN(kMinHeight, kMaxHeight);
    double radius = randMToN(kMinRadius, kMaxRadius);
    total_volume = total_volume + 3.1415 * radius * radius * height;
    // First select its pose.
    Eigen::Vector3d position(randMToN(free_space_bounds.x(), size.x() - free_space_bounds.x()),
                             randMToN(free_space_bounds.y(), size.y() - free_space_bounds.y()), height / 2.0);

    /*    system("rosrun gazebo_ros spawn_model -file `rospack find acl_sim`/urdf/window.urdf -urdf -x " + str(x) + " -y
       " + str(y) + " -z " + str(z) + " -R " + str(roll) + " -P " + str(pitch) + " -Y " + str(yaw) + " -model gate_" +
               str(uniform(1, 10000)));*/

    double x = position[0];
    double y = position[1];
    double z = position[2];

    spawner.spawnCube(std::to_string(i), "world", x, y, z, 0, 0, 0, 1, radius, height, 1, 2);

    std::string x_string = std::to_string(x);
    std::string y_string = std::to_string(y);

    std::string z_string = std::to_string(z);
    std::string i_string = std::to_string(i);

    /*    system(("rosrun gazebo_ros spawn_model -file `rospack find acl_sim`/models/cylinder/model.sdf -sdf -x " +
       x_string + " -y " + y_string + " -z 0 -model pole_" + i_string) .c_str());*/

    /*    world_.addObject(std::unique_ptr<voxblox::Object>(
            new voxblox::Cylinder(position.cast<float>(), radius, height, voxblox::Color::Gray())));*/

    printf("SEED=%d\n", seed);
  }

  printf("Total Volume=%f\n", total_volume);
  printf("Usable_area=%f\n", usable_area);

  ros::spin();
  return 0;
}
