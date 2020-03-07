/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#include <ros/ros.h>

#include <Eigen/Core>

#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/mpc_local_planner_ros.h>
#include <mpc_local_planner/utils/publisher.h>
#include <teb_local_planner/obstacles.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <memory>

namespace mpc = mpc_local_planner;

class TestMpcOptimNode
{
 public:
    TestMpcOptimNode() = default;

    void start(ros::NodeHandle& nh);

 protected:
    void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame,
                                 interactive_markers::InteractiveMarkerServer* marker_server);
    void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
    void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
    void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);

    teb_local_planner::ObstContainer _obstacles;
    int _no_fixed_obstacles;
    std::vector<teb_local_planner::PoseSE2> _via_points;
};

void TestMpcOptimNode::start(ros::NodeHandle& nh)
{
    std::string map_frame = "map";

    // interactive marker server for simulated dynamic obstacles
    interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

    // configure obstacles
    _obstacles.push_back(boost::make_shared<teb_local_planner::PointObstacle>(-3, 1));
    _obstacles.push_back(boost::make_shared<teb_local_planner::PointObstacle>(6, 2));
    _obstacles.push_back(boost::make_shared<teb_local_planner::PointObstacle>(4, 0.1));

    // Add interactive markers
    for (int i = 0; i < (int)_obstacles.size(); ++i)
    {
        // Add interactive markers for all point obstacles
        boost::shared_ptr<teb_local_planner::PointObstacle> pobst = boost::dynamic_pointer_cast<teb_local_planner::PointObstacle>(_obstacles[i]);
        if (pobst)
        {
            CreateInteractiveMarker(pobst->x(), pobst->y(), i, map_frame, &marker_server);
        }
    }
    marker_server.applyChanges();
    _no_fixed_obstacles = (int)_obstacles.size();

    // setup callback for custom obstacles
    ros::Subscriber custom_obst_sub = nh.subscribe("obstacles", 1, &TestMpcOptimNode::CB_customObstacle, this);

    // setup callback for clicked points (in rviz) that are considered as via-points
    ros::Subscriber clicked_points_sub = nh.subscribe("/clicked_point", 5, &TestMpcOptimNode::CB_clicked_points, this);

    // setup callback for via-points (callback overwrites previously set via-points)
    ros::Subscriber via_points_sub = nh.subscribe("via_points", 1, &TestMpcOptimNode::CB_via_points, this);

    // Setup robot shape model
    teb_local_planner::RobotFootprintModelPtr robot_model = mpc_local_planner::MpcLocalPlannerROS::getRobotFootprintFromParamServer(nh);

    mpc_local_planner::Controller controller;
    if (!controller.configure(nh, _obstacles, robot_model, _via_points))
    {
        ROS_ERROR("Controller configuration failed.");
        return;
    }

    mpc::Publisher publisher(nh, controller.getRobotDynamics(), map_frame);

    teb_local_planner::PoseSE2 x0(0, 0, 0);
    teb_local_planner::PoseSE2 xf(5, 2, 0);

    corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
    corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();

    geometry_msgs::Twist vel;

    bool success = false;

    ros::Rate rate(20);
    while (ros::ok())
    {
        success = controller.step(x0, xf, vel, rate.expectedCycleTime().toSec(), ros::Time::now(), u_seq, x_seq);

        if (success)
            publisher.publishLocalPlan(*x_seq);
        else
            ROS_ERROR("OCP solving failed.");

        publisher.publishObstacles(_obstacles);
        publisher.publishRobotFootprintModel(x0, *robot_model);
        publisher.publishViaPoints(_via_points);
        ros::spinOnce();
        rate.sleep();
    }
}

void TestMpcOptimNode::CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame,
                                               interactive_markers::InteractiveMarkerServer* marker_server)
{
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker i_marker;
    i_marker.header.frame_id = frame;
    i_marker.header.stamp    = ros::Time::now();
    std::ostringstream oss;
    // oss << "obstacle" << id;
    oss << id;
    i_marker.name               = oss.str();
    i_marker.description        = "Obstacle";
    i_marker.pose.position.x    = init_x;
    i_marker.pose.position.y    = init_y;
    i_marker.pose.orientation.w = 1.0f;  // make quaternion normalized

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type               = visualization_msgs::Marker::CUBE;
    box_marker.id                 = id;
    box_marker.scale.x            = 0.2;
    box_marker.scale.y            = 0.2;
    box_marker.scale.z            = 0.2;
    box_marker.color.r            = 0.5;
    box_marker.color.g            = 0.5;
    box_marker.color.b            = 0.5;
    box_marker.color.a            = 1.0;
    box_marker.pose.orientation.w = 1.0f;  // make quaternion normalized

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    i_marker.controls.push_back(box_control);

    // create a control which will move the box, rviz will insert 2 arrows
    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name             = "move_x";
    move_control.orientation.w    = 0.707107f;
    move_control.orientation.x    = 0;
    move_control.orientation.y    = 0.707107f;
    move_control.orientation.z    = 0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

    // add the control to the interactive marker
    i_marker.controls.push_back(move_control);

    // add the interactive marker to our collection
    marker_server->insert(i_marker);
    marker_server->setCallback(i_marker.name, boost::bind(&TestMpcOptimNode::CB_obstacle_marker, this, boost::placeholders::_1));
}

void TestMpcOptimNode::CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    std::stringstream ss(feedback->marker_name);
    unsigned int index;
    ss >> index;

    if (index >= _no_fixed_obstacles) return;
    teb_local_planner::PointObstacle* pobst = static_cast<teb_local_planner::PointObstacle*>(_obstacles.at(index).get());
    pobst->position()                       = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
}

void TestMpcOptimNode::CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
    // resize such that the vector contains only the fixed obstacles specified inside the main function
    _obstacles.resize(_no_fixed_obstacles);

    // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)
    for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
    {
        if (obst_msg->obstacles.at(i).polygon.points.size() == 1)
        {
            if (obst_msg->obstacles.at(i).radius == 0)
            {
                _obstacles.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(
                    obst_msg->obstacles.at(i).polygon.points.front().x, obst_msg->obstacles.at(i).polygon.points.front().y)));
            }
            else
            {
                _obstacles.push_back(teb_local_planner::ObstaclePtr(
                    new teb_local_planner::CircularObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y, obst_msg->obstacles.at(i).radius)));
            }
        }
        else
        {
            teb_local_planner::PolygonObstacle* polyobst = new teb_local_planner::PolygonObstacle;
            for (size_t j = 0; j < obst_msg->obstacles.at(i).polygon.points.size(); ++j)
            {
                polyobst->pushBackVertex(obst_msg->obstacles.at(i).polygon.points[j].x, obst_msg->obstacles.at(i).polygon.points[j].y);
            }
            polyobst->finalizePolygon();
            _obstacles.push_back(teb_local_planner::ObstaclePtr(polyobst));
        }

        if (!_obstacles.empty()) _obstacles.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
    }
}

void TestMpcOptimNode::CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
    // we assume for simplicity that the fixed frame is already the map/planning frame
    // consider clicked points as via-points
    _via_points.emplace_back(point_msg->point.x, point_msg->point.y, 0.0);
    ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
}

void TestMpcOptimNode::CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
    ROS_INFO_ONCE("Via-points received. This message is printed once.");
    _via_points.clear();
    for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
    {
        _via_points.emplace_back(pose.pose.position.x, pose.pose.position.y, 0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_optim_node");
    ros::NodeHandle n("~");

    TestMpcOptimNode mpc_test;
    mpc_test.start(n);

    return 0;
}
