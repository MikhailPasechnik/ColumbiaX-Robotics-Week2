#include <ros/ros.h>
#include <tf/LinearMath/Scalar.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/ColorRGBA.h>

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

template<typename T> T create_triple(tfScalar x, tfScalar y, tfScalar z)
{
    T p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

template<typename T> T create_quadruple(tfScalar x, tfScalar y, tfScalar z, tfScalar w)
{
    T t;
    t.x = x;
    t.y = y;
    t.z = z;
    t.w = w;
    return t;
}

ColorRGBA create_color(float r, float g, float b, float a)
{
    ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (ros::ok())
  {
    visualization_msgs::Marker object;
    object.header.frame_id = "/object_frame";
    object.header.stamp = ros::Time::now();
    object.ns = "project_2";
    object.id = 0;
    object.type = visualization_msgs::Marker::CYLINDER;
    object.action = visualization_msgs::Marker::ADD;
    object.pose.position = create_triple<Point>(0, 0, 0);
    object.pose.orientation = create_quadruple<Quaternion>(0, 0, 0, 1);
    object.scale = create_triple<Vector3>(0.3, 0.3, 0.3);
    object.color = create_color(0.0, 1.0, 0.0, 0.3);
    object.lifetime = ros::Duration();
    marker_pub.publish(object);

    ros::Duration(1).sleep();

    visualization_msgs::Marker robot;
    robot.header.frame_id = "/robot_frame";
    robot.header.stamp = ros::Time::now();
    robot.ns = "project_2";
    robot.id = 1;
    robot.type = visualization_msgs::Marker::CUBE;
    robot.action = visualization_msgs::Marker::ADD;
    robot.pose.orientation = create_quadruple<Quaternion>(0, 0, 0, 1);
    robot.scale = create_triple<Vector3>(0.3, 0.3, 0.3);
    robot.color = create_color(0.0, 1.0, 0.0, 0.3);
    robot.lifetime = ros::Duration();
    marker_pub.publish(robot);

    ros::Duration(1).sleep();

    visualization_msgs::Marker camera;
    camera.header.frame_id = "/camera_frame";
    camera.header.stamp = ros::Time::now();
    camera.ns = "project_2";
    camera.id = 2;
    camera.type = visualization_msgs::Marker::ARROW;
    camera.action = visualization_msgs::Marker::ADD;

    camera.pose.orientation = create_quadruple<Quaternion>(0, 0, 0, 1);
    camera.scale = create_triple<Vector3>(0.4, 0.1, 0.1);
    camera.color = create_color(1.0, 0.0, 0.0, 0.7);
    camera.lifetime = ros::Duration();
    marker_pub.publish(camera);

    ros::Duration(0.25).sleep();
  }
}
