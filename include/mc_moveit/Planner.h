#pragma once

#include <mc_moveit/BSplineTrajectoryTask.h>
#include <mc_moveit/PostureTrajectoryTask.h>
#include <mc_moveit/config.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>
#include <moveit_msgs/msg/detail/attached_collision_object__struct.hpp>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <rclcpp/publisher.hpp>

#include <mc_rbdyn/RobotFrame.h>

#include <RBDyn/parsers/common.h>

#include <mc_rtc_ros/ros.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

using MoveItCpp = moveit_cpp::MoveItCpp;
using PlanningComponent = moveit_cpp::PlanningComponent;

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <future>

namespace octomap
{
struct OcTree;
}

namespace mc_moveit
{

struct MC_MOVEIT_DLLAPI PlannerConfig
{
  /** ROS namespace for MoveIt. This should be unique per-instance if you want to use multiple planner instances */
  std::string ns = "mc_moveit";
  /** Configuration file containing the planner parameters */
  mc_rtc::Configuration config = mc_rtc::Configuration(DEFAULT_PLANNER_CONFIG);

  inline PlannerConfig() = default;
  inline PlannerConfig(const std::string & ns_in) : ns(ns_in) {}
};

/** Provides an interface to MoveIt from an mc_rtc robot
 *
 * The interface translates mc_rtc concepts into MoveIt concept and provides a convenient interface to request plans to
 * MoveIt and get them in a usable form for an mc_rtc controller
 *
 */
struct MC_MOVEIT_DLLAPI Planner
{
  /** Constructor
   *
   * Constructing the planner interface is relatively lenghty. If this should be invoked inside a controller you should
   * prefer the async helper: \ref make_planner
   *
   * \param robot Robot that will be used by the planner
   *
   * \param ef_body End-effector body that will be used by the planner, this body should figure in the robot's URDF
   * otherwise the planner cannot work. However through the interface you can request plans for frames that do not
   * appear in the URDF
   *
   * \param ns ROS namespace for MoveIt. If you are using multiple planners instances this should be unique for each
   * instance
   */
  Planner(const mc_rbdyn::Robot & robot, const std::string & ef_body, const PlannerConfig & = {});

  struct Obstacle
  {
    rbd::parsers::Visual object;
    sva::PTransformd pose;
  };
  struct AttachedObject
  {
    rbd::parsers::Visual object;
    sva::PTransformd pose;
    std::string link_name;
  };

  /**
   * @brief Attach an object to a robot frame
   *
   * @param object_name Name of the object to attach (unique)
   * @param link_name Name of the robot's link to which the object will be attached
   * @param object Visual description of the object
   * @param X_link_object Transformation between the robot's link (link_name) and the origin of the object's visual
   */
  void attach_object(const std::string & object_name,
                     const std::string link_name,
                     const rbd::parsers::Visual & object,
                     const sva::PTransformd & X_link_object);
  /**
   * @brief Detach an object from the robot
   *
   * @param object_name Name of the object previously added by attach_object()
   * @see attach_object
   */
  void detach_object(const std::string & object_name);

  inline const AttachedObject & attached_object(const std::string & object_name) const
  {
    return attached_objects_.at(object_name);
  }

  /** Add a collision object to the planning
   *
   * The name of \p object uniquely identify the object. Adding two objects with the same name will make the first one
   * disappear from MoveIt POV
   *
   * \param object Describe the collision object
   *
   * \param X_0_object Initial position of the collision object in world frame
   */
  void add_obstacle(const rbd::parsers::Visual & object,
                    const sva::PTransformd & X_0_object = sva::PTransformd::Identity());

  /** Update an obstacle location only
   *
   * \param name Name of the object to be updated
   *
   * \param X_0_object New position of the object in world frame
   */
  void update_obstacle(const std::string & name, const sva::PTransformd & X_0_object);

  /** Remove an obstacle
   *
   * \param name Name of the obstacle to be removed
   */
  void remove_obstacle(const std::string & name);

  /** Remove all obstacles */
  void clear_obstacles();

  /** Sets an octomap as an obstacle in the planner */
  void set_octomap(const octomap::OcTree & octomap, const sva::PTransformd & X_0_base);

  /** Returns an obstacle as stored in the planner
   *
   * \param name Name of the object
   *
   * \returns An invalid obstacle if \p name is not a known obstacle
   */
  const Obstacle & get_obstacle(const std::string & name);

  /** The trajectory returned by a plan */
  struct MC_MOVEIT_DLLAPI Trajectory
  {
    struct MC_MOVEIT_DLLAPI TimedPose
    {
      double time;
      sva::PTransformd pose;
    };
    /** Error provided by MoveIt
     * - 1 indicates success
     * - 10 busy planner
     * - Other values indicate an error, see
     * http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/MoveItErrorCodes.html for a brief description
     */
    decltype(moveit_msgs::msg::MoveItErrorCodes::val) error;
    /** List of waypoints in this trajectory including the first and final waypoints */
    std::vector<TimedPose> waypoints;
    /** List of posture waypoints configuration including the first and final waypoints */
    std::vector<TimedPosture> postures;
    /** Setup a BSplineTrajectoryTask with the points in this trajectory
     *
     * \param frame Name of the robot frame used by the task
     *
     * \param stiffness Task stiffness
     *
     * \param weight Task weight
     */
    std::shared_ptr<mc_moveit::BSplineTrajectoryTask> setup_bspline_task(const mc_rbdyn::RobotFrame & frame,
                                                                         double stiffness,
                                                                         double weight) const;
    /** Setup a PostureTrajectoryTask with the points in this trajectory
     *
     * \param robot Robot controller by this task
     *
     * \param stiffness Task stiffness
     *
     * \param weight Task weight
     */
    std::shared_ptr<mc_moveit::PostureTrajectoryTask> setup_posture_task(const mc_solver::QPSolver & solver,
                                                                         mc_rbdyn::Robot & robot,
                                                                         double stiffness,
                                                                         double weight) const;
  };

  /** Try to plan a motion from the current position of \p frame to \p goal
   *
   * If X_b_s is the transformation from the provided frame to the body provided in the constructor, the planner will
   * compute a plan such for the body to reach X_0_b' such that X_b_s * X_0_b' = X_0_goal. Hence the transformation from
   * \p frame to the body should be static otherwise the result won't make a lot of sense.
   *
   * For use in a controller, prefer \ref plan_async as this function can take a while to compute
   *
   * \param frame Frame of the robot upon which this planner acts
   *
   * \param X_0_goal Planning goal in world frame
   *
   */
  Trajectory plan(const mc_rbdyn::RobotFrame & frame, const sva::PTransformd & X_0_goal);

  /** Try to plan a motion from the current configuration to the target configuration
   *
   * \param frame Frame of the robot upon which this planner acts
   *
   * \param target Target configuration of the robot
   */
  Trajectory plan(const mc_rbdyn::RobotFrame & frame, const std::map<std::string, std::vector<double>> & target);

  /** Async version of \ref plan */
  std::future<Trajectory> plan_async(const mc_rbdyn::RobotFrame & frame, const sva::PTransformd & X_0_goal);

  /** Async version of \ref plan */
  std::future<Trajectory> plan_async(const mc_rbdyn::RobotFrame & frame,
                                     const std::map<std::string, std::vector<double>> & target);

private:
  rclcpp::Node::SharedPtr nh_;
  std::string body_;
  tf2_ros::StaticTransformBroadcaster tf_static_caster_;
  std::atomic<bool> busy_{false};
  std::shared_ptr<MoveItCpp> moveit_cpp_ptr_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  std::shared_ptr<PlanningComponent> planning_;
  moveit::core::RobotModelConstPtr robot_model_;

  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_objects_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  std::map<std::string, Obstacle> obstacles_;
  std::map<std::string, AttachedObject> attached_objects_;
  bool plan_with_cartesian_goal_ = false;

  void remove_obstacle(const std::map<std::string, Obstacle>::iterator & it);
  void detach_object(const std::map<std::string, AttachedObject>::iterator & it);

  Trajectory do_plan(const mc_rbdyn::RobotFrame & frame);
};

using PlannerPtr = std::unique_ptr<Planner>;

MC_MOVEIT_DLLAPI std::future<PlannerPtr> make_planner(const mc_rbdyn::Robot & robot,
                                                      const std::string & ef_body,
                                                      const PlannerConfig & = {});

} // namespace mc_moveit
