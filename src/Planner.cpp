#include <mc_moveit/Planner.h>

#include <mc_rbdyn/Robot.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap_with_pose.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <mc_rtc_ros/ros.h>
#include <octomap/OcTree.h>
#include <rclcpp/node_options.hpp>

namespace mc_moveit
{

static void PtToMsg(const sva::PTransformd & pt, geometry_msgs::msg::Pose & msg)
{
  msg.position.x = pt.translation().x();
  msg.position.y = pt.translation().y();
  msg.position.z = pt.translation().z();
  auto q = Eigen::Quaterniond(pt.rotation().transpose()).normalized();
  q.normalize();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
}

static geometry_msgs::msg::Pose PtToMsg(const sva::PTransformd & pt)
{
  geometry_msgs::msg::Pose pose;
  PtToMsg(pt, pose);
  return pose;
}

// FIXME This is not complete enough for sure
static void configToParam(rclcpp::Node & nh,
                          const std::string & ns,
                          const std::string & k,
                          const mc_rtc::Configuration & value)
{
  if(value.isString()) { nh.set_parameter({ns + k, value.operator std::string()}); }
  else if(value.isNumeric()) { nh.set_parameter({ns + k, value.operator double()}); }
  else { nh.set_parameter({ns + k, value.operator bool()}); }
}

static constexpr auto DEFAULT_GROUP = "default_group";

static std::string make_simple_srdf(const mc_rbdyn::Robot & robot, const std::string & ef_body)
{
  const auto & bodies = robot.mb().bodies();
  std::string srdf = fmt::format(R"(<?xml version="1.0" ?>
  <robot name="{}">
)",
                                 robot.name());
  srdf += fmt::format(R"(  <group name="{}">
    <chain base_link="{}" tip_link="{}" />
  </group>
  <virtual_joint name="virtual_joint" type="floating" parent_frame="robot_map" child_link="{}" />
)",
                      DEFAULT_GROUP, bodies[0].name(), ef_body, bodies[0].name());
  const auto & collisions = robot.module().commonSelfCollisions();
  auto has_collision = [&](const std::string & b1, const std::string & b2)
  {
    auto it = std::find_if(collisions.begin(), collisions.end(), [&b1, &b2](const mc_rbdyn::Collision & c)
                           { return (c.body1 == b1 && c.body2 == b2) || (c.body1 == b2 && c.body2 == b1); });
    return it != collisions.end();
  };
  for(size_t i = 0; i < bodies.size(); ++i)
  {
    const auto & b1 = bodies[i].name();
    for(size_t j = i + 1; j < bodies.size(); ++j)
    {
      const auto & b2 = bodies[j].name();
      if(!has_collision(b1, b2))
      {
        srdf += fmt::format(R"(  <disable_collisions link1="{}" link2="{}" reason="Default" />)", b1, b2);
        srdf += "\n";
      }
    }
  }
  srdf += "</robot>";
  return srdf;
}

static void visual_to_msg(const rbd::parsers::Visual & visual, moveit_msgs::msg::CollisionObject & object)
{
  auto box_to_msg = [&]()
  {
    const auto & box = boost::get<rbd::parsers::Geometry::Box>(visual.geometry.data);
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape.BOX;
    shape.dimensions.resize(3);
    shape.dimensions[0] = box.size.x();
    shape.dimensions[1] = box.size.y();
    shape.dimensions[2] = box.size.z();
    object.primitives.push_back(shape);
  };
  auto cylinder_to_msg = [&]()
  {
    const auto & cyl = boost::get<rbd::parsers::Geometry::Cylinder>(visual.geometry.data);
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape.CYLINDER;
    shape.dimensions.resize(2);
    shape.dimensions[shape.CYLINDER_HEIGHT] = cyl.length;
    shape.dimensions[shape.CYLINDER_RADIUS] = cyl.radius;
    object.primitives.push_back(shape);
  };
  auto sphere_to_msg = [&]()
  {
    const auto & sph = boost::get<rbd::parsers::Geometry::Sphere>(visual.geometry.data);
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape.SPHERE;
    shape.dimensions.resize(1);
    shape.dimensions[0] = sph.radius;
    object.primitives.push_back(shape);
  };
  auto mesh_to_msg = [&]()
  {
    const auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(visual.geometry.data);
    auto mesh_data = shapes::createMeshFromResource(mesh.filename, mesh.scaleV);
    object.meshes.emplace({});
    shape_msgs::msg::Mesh & shape = object.meshes.back();
    shape.triangles.resize(mesh_data->triangle_count);
    for(size_t i = 0; i < mesh_data->triangle_count; ++i)
    {
      for(size_t j = 0; j < 3; ++j) { shape.triangles[i].vertex_indices[j] = mesh_data->triangles[3 * i + j]; }
    }
    shape.vertices.resize(mesh_data->vertex_count);
    for(size_t i = 0; i < mesh_data->vertex_count; ++i)
    {
      shape.vertices[i].x = mesh_data->vertices[3 * i + 0];
      shape.vertices[i].y = mesh_data->vertices[3 * i + 1];
      shape.vertices[i].z = mesh_data->vertices[3 * i + 2];
    }
  };
  switch(visual.geometry.type)
  {
    case rbd::parsers::Geometry::BOX:
      box_to_msg();
      return;
    case rbd::parsers::Geometry::CYLINDER:
      cylinder_to_msg();
      return;
    case rbd::parsers::Geometry::SPHERE:
      sphere_to_msg();
      return;
    case rbd::parsers::Geometry::MESH:
      mesh_to_msg();
      return;
    default:
      mc_rtc::log::error("Visual type is not handled yet");
      return;
  }
}

Planner::Planner(const mc_rbdyn::Robot & robot, const std::string & ef_body, const PlannerConfig & config)
: rclcpp::Node(config.ns, rclcpp::NodeOptions{}.allow_undeclared_parameters(true)), body_(ef_body)
{
}

Planner::~Planner()
{
  if(spin_thread_.joinable()) { spin_thread_.join(); }
}

void Planner::initialize(const mc_rbdyn::Robot & robot, const std::string & ef_body, const PlannerConfig & config)
{
  // We have to spin within the planner to handle parameters
  spin_thread_ = std::thread([this]() { rclcpp::spin(shared_from_this()); });

  tf_static_caster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

  if(!robot.hasBody(ef_body))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_moveit] {} has no body named {}", robot.name(), ef_body);
  }
  auto mc_rtc_nh = mc_rtc::ROSBridge::get_node_handle();
  if(!mc_rtc_nh)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_moveit] mc_rtc ROS plugin must be enabled for this controller to work");
  }

  std::string robot_tf_prefix = "/control";
  if(robot.robotIndex() != 0) { robot_tf_prefix = fmt::format("/control/env_{}", robot.robotIndex()); }

  // FIXME This should not be needed but it fixes many TF warnings from MoveIt as it is not looking into the prefixed TF
  // for transformation involving the robot's base
  geometry_msgs::msg::TransformStamped base_to_base;
  base_to_base.header.frame_id = fmt::format("{}/{}", robot_tf_prefix, robot.mb().body(0).name());
  base_to_base.child_frame_id = robot.mb().body(0).name();
  base_to_base.transform.rotation.w = 1.0;
  tf_static_caster_->sendTransform(base_to_base);

  // FIXME To support multiple planners for a single robot we should nest all this settings in a proper namespace and
  // copy some settings over from the robot_tf_prefix

  // FIXME Mimic/Support loading a lot of these settings via roslaunch
  set_parameter({fmt::format("{}/robot_description_semantic", robot_tf_prefix), make_simple_srdf(robot, ef_body)});
  {
    std::string ns = fmt::format("{}/robot_description_kinematics/{}/", robot_tf_prefix, DEFAULT_GROUP);
    set_parameters({{ns + "kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin"},
                    {ns + "kinematics_solver_search_resolution", 0.005},
                    {ns + "kinematics_solver_timeout", 500 * 0.005}});
  }
  {
    std::string ns = "planning_scene_monitor_options/";
    set_parameters(
        {{ns + "name", config.ns},
         {ns + "robot_description", fmt::format("{}/robot_description", robot_tf_prefix)},
         {ns + "joint_state_topic", fmt::format("{}/joint_states", robot_tf_prefix)},
         {ns + "attached_collision_object_topic",
          fmt::format("{}/{}", config.ns,
                      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC)},
         {ns + "monitored_planning_scene_topic",
          fmt::format("{}/{}", config.ns,
                      planning_scene_monitor::PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC)},
         {ns + "publish_planning_scene_topic",
          fmt::format("{}/{}", config.ns,
                      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC)}});

    ns = "planning_pipelines/";
    std::vector<std::string> pipeline_names = config.config("pipeline_names");
    set_parameters({{ns + "pipeline_names", pipeline_names}, {ns + "namespace", config.ns}});

    for(const auto & pipeline : pipeline_names)
    {
      ns = fmt::format("{}/", pipeline);
      auto pipec = config.config(pipeline);
      set_parameter({ns + "planning_plugin", pipec("planning_plugin").operator std::string()});
      std::vector<std::string> planning_adapters = pipec("request_adapters", std::vector<std::string>{});
      if(planning_adapters.size())
      {
        std::string request_adapters = planning_adapters[0];
        for(size_t i = 1; i < planning_adapters.size(); ++i)
        {
          const auto & pa = planning_adapters[i];
          request_adapters += " ";
          request_adapters += pa;
        }
        set_parameter({ns + "request_adapters", request_adapters});
      }
      std::vector<std::string> keys = pipec.keys();
      for(const auto & k : keys)
      {
        if(k == "request_adapters" || k == "planning_plugin") { continue; }
        auto value = pipec(k);
        configToParam(*this, ns, k, value);
      }
    }

    set_parameter({"moveit_manage_controllers", false});

    ns = "plan_request_params/";
    {
      auto pr_params = config.config("plan_request_params");
      plan_with_cartesian_goal_ = config.config(pr_params("planning_pipeline"))("plan_with_cartesian_goal");
      for(const auto & k : pr_params.keys())
      {
        auto value = pr_params(k);
        configToParam(*this, ns, k, value);
      }
    }
  }

  auto opts = MoveItCpp::Options{shared_from_this()};
  {
    std::string ns = "planning_scene_monitor_options/";
    // opts.planning_scene_monitor_options.robot_description = ns + "robot_description";
  }

  moveit_cpp_ptr_ = std::make_shared<MoveItCpp>(Node::shared_from_this(), opts);

  monitor_ = moveit_cpp_ptr_->getPlanningSceneMonitor();
  monitor_->providePlanningSceneService(
      fmt::format("{}/{}", config.ns, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE));
  monitor_->startWorldGeometryMonitor(
      fmt::format("{}/{}", config.ns, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC),
      fmt::format("{}/{}", config.ns, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC),
      true /* octomap monitor */);

  planning_ = std::make_shared<PlanningComponent>(DEFAULT_GROUP, moveit_cpp_ptr_);

  robot_model_ = moveit_cpp_ptr_->getRobotModel();

  obstacles_publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC, 256);
  attached_objects_publisher_ = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC, 256);
  planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
      fmt::format("{}", planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC), 10);
}

void Planner::attach_object(const std::string & object_name,
                            const std::string link_name,
                            const rbd::parsers::Visual & object,
                            const sva::PTransformd & X_link_object)
{
  // First remove the obstacle from the environment if it was there
  remove_obstacle(object_name);

  attached_objects_[object_name] = {object, X_link_object, link_name};

  // Then attach the object to the robot
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = link_name;
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = link_name;
  /* The id of the object */
  attached_object.object.id = object_name;
  /* Pose of the attached object */
  attached_object.object.pose = PtToMsg(X_link_object);

  moveit_msgs::msg::CollisionObject object_msg;
  visual_to_msg(object, attached_object.object);
  attached_object.object.operation = attached_object.object.ADD;
  attached_objects_publisher_->publish(attached_object);
}

void Planner::detach_object(const std::string & object_name)
{
  auto it = attached_objects_.find(object_name);
  if(it == attached_objects_.end()) { return; }
  detach_object(it);
}

void Planner::detach_object(const std::map<std::string, AttachedObject>::iterator & it)
{
  {
    moveit_msgs::msg::AttachedCollisionObject msg;
    msg.object.id = it->first;
    msg.link_name = it->second.link_name;
    msg.object.operation = msg.object.REMOVE;
    attached_objects_publisher_->publish(msg);
    attached_objects_.erase(it);
  }

  { // XXX this is done to handle detached objects remaining in the scene
    moveit_msgs::msg::CollisionObject msg;
    msg.header.frame_id = monitor_->getPlanningScene()->getPlanningFrame();
    msg.id = it->first;
    msg.operation = msg.REMOVE;
    obstacles_publisher_->publish(msg);
  }
}

void Planner::add_obstacle(const rbd::parsers::Visual & object, const sva::PTransformd & X_0_object)
{
  mc_rtc::log::info("[Planner] Adding obstacle {}", object.name);
  obstacles_[object.name] = {object, X_0_object};
  moveit_msgs::msg::CollisionObject msg;
  msg.header.frame_id = monitor_->getPlanningScene()->getPlanningFrame();
  msg.id = object.name;
  visual_to_msg(object, msg);
  msg.pose = PtToMsg(object.origin * X_0_object);
  msg.operation = msg.ADD;
  obstacles_publisher_->publish(msg);
  planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr_->getPlanningSceneMonitor());
  scene->processCollisionObjectMsg(msg);
}

void Planner::update_obstacle(const std::string & object, const sva::PTransformd & X_0_object)
{
  auto it = obstacles_.find(object);
  if(it == obstacles_.end())
  {
    mc_rtc::log::error("[mc_moveit] Cannot update position of an obstacle that was not added");
    return;
  }
  it->second.pose = X_0_object;
  moveit_msgs::msg::CollisionObject msg;
  msg.header.frame_id = monitor_->getPlanningScene()->getPlanningFrame();
  msg.id = object;
  msg.pose = PtToMsg(it->second.object.origin * X_0_object);
  msg.operation = msg.MOVE;
  obstacles_publisher_->publish(msg);
  planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr_->getPlanningSceneMonitor());
  scene->processCollisionObjectMsg(msg);
}

void Planner::remove_obstacle(const std::string & object)
{
  auto it = obstacles_.find(object);
  if(it == obstacles_.end()) { return; }

  remove_obstacle(it);
}

void Planner::remove_obstacle(const std::map<std::string, Obstacle>::iterator & it)
{
  moveit_msgs::msg::CollisionObject msg;
  msg.header.frame_id = monitor_->getPlanningScene()->getPlanningFrame();
  msg.id = it->first;
  msg.operation = msg.REMOVE;
  obstacles_publisher_->publish(msg);
  planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr_->getPlanningSceneMonitor());
  scene->processCollisionObjectMsg(msg);
  obstacles_.erase(it);
  mc_rtc::log::success("[Planner] Removed obstacle {}", it->first);
}

void Planner::clear_obstacles()
{
  while(!obstacles_.empty()) { remove_obstacle(obstacles_.begin()); }
}

auto Planner::get_obstacle(const std::string & name) -> const Obstacle &
{
  auto it = obstacles_.find(name);
  if(it == obstacles_.end())
  {
    static Obstacle default_{};
    return default_;
  }
  return it->second;
}

void Planner::set_octomap(const octomap::OcTree & octomap_, const sva::PTransformd & base_)
{
  auto planning_scene_msg = moveit_msgs::msg::PlanningScene{};
  monitor_->getPlanningScene()->getPlanningSceneMsg(planning_scene_msg);
  auto & octomap = planning_scene_msg.world.octomap;
  octomap.header.stamp = this->now();
  octomap.header.frame_id = "robot_map";
  octomap_msgs::fullMapToMsg(octomap_, octomap.octomap);

  octomap.origin.position.x = base_.translation().x();
  octomap.origin.position.y = base_.translation().y();
  ;
  octomap.origin.position.z = base_.translation().z();
  ;
  Eigen::Quaterniond q(base_.rotation().inverse());
  octomap.origin.orientation.w = q.w();
  octomap.origin.orientation.x = q.x();
  octomap.origin.orientation.y = q.y();
  octomap.origin.orientation.z = q.z();

  planning_scene_msg.is_diff = true;
  planning_scene_publisher_->publish(planning_scene_msg);
  planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr_->getPlanningSceneMonitor());
  scene->processOctomapMsg(octomap);
}

auto Planner::plan(const mc_rbdyn::RobotFrame & frame, const sva::PTransformd & X_0_goal) -> Trajectory
{
  if(busy_)
  {
    mc_rtc::log::error("[mc_moveit] Plan requested failed as the planner is busy");
    return Trajectory{10, {}};
  }
  planning_->setStartStateToCurrentState();
  auto target = frame.X_b_f().inv() * X_0_goal;
  if(plan_with_cartesian_goal_)
  {
    // Set cartesian space goal
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = robot_model_->getModelFrame();
    PtToMsg(target, goal.pose);
    planning_->setGoal(goal, body_);
  }
  else
  {
    // Set joint space goal from MoveIt IK
    auto goal_state = *(moveit_cpp_ptr_->getCurrentState());
    geometry_msgs::msg::Pose goal_pose;
    PtToMsg(target, goal_pose);
    goal_state.setFromIK(robot_model_->getJointModelGroup(DEFAULT_GROUP), goal_pose);
    planning_->setGoal(goal_state);
  }
  return do_plan(frame);
}

auto Planner::plan(const mc_rbdyn::RobotFrame & frame, const std::map<std::string, std::vector<double>> & target)
    -> Trajectory
{
  if(busy_)
  {
    mc_rtc::log::error("[mc_moveit] Plan requested failed as the planner is busy");
    return Trajectory{10, {}};
  }
  planning_->setStartStateToCurrentState();
  auto goal_state = *(moveit_cpp_ptr_->getCurrentState());
  for(const auto & [name, config] : target)
  {
    if(config.size() == 0) { continue; }
    if(goal_state.getJointModel(name)->getVariableCount() != config.size())
    {
      mc_rtc::log::error("Invalid posture target for {}", name);
      return {10, {}};
    }
    goal_state.setJointPositions(name, config.data());
    planning_->setGoal(goal_state);
  }
  return do_plan(frame);
}

auto Planner::do_plan(const mc_rbdyn::RobotFrame & frame) -> Trajectory
{
  auto solution = planning_->plan();
  Trajectory out;
  out.error = solution.error_code.val;
  if(!solution)
  {
    mc_rtc::log::error("[mc_moveit] Failed to find a solution, check above for more details");
    mc_rtc::log::info("MoveIt error code: {}", solution.error_code.val);
    return out;
  }
  const auto & trajectory = solution.trajectory;
  out.waypoints.reserve(trajectory->getWayPointCount());
  out.postures.reserve(trajectory->getWayPointCount());
  auto state = std::make_shared<moveit::core::RobotState>(*(moveit_cpp_ptr_->getCurrentState()));
  auto group = state->getJointModelGroup(DEFAULT_GROUP);
  for(size_t i = 0; i < trajectory->getWayPointCount(); ++i)
  {
    auto wp_affine3d = trajectory->getWayPoint(i).getFrameTransform(body_);
    sva::PTransformd wp{wp_affine3d.rotation().transpose(), wp_affine3d.translation()};
    double t = trajectory->getWayPointDurationFromStart(i);
    out.waypoints.push_back({t, frame.X_b_f() * wp});
    mc_rtc::log::info("Get state at t: {}", t);
    trajectory->getStateAtDurationFromStart(t, state);
    std::map<std::string, std::vector<double>> posture;
    for(const auto & j : group->getJointModelNames())
    {
      if(group->getJointModel(j)->getStateSpaceDimension() == 1) { posture[j] = {*state->getJointPositions(j)}; }
    }
    out.postures.push_back({t, posture});
  }
  return out;
}

auto Planner::plan_async(const mc_rbdyn::RobotFrame & frame, const sva::PTransformd & X_0_goal)
    -> std::future<Trajectory>
{
  return std::async(std::launch::async, [&frame, X_0_goal, this]() { return plan(frame, X_0_goal); });
}

auto Planner::plan_async(const mc_rbdyn::RobotFrame & frame, const std::map<std::string, std::vector<double>> & target)
    -> std::future<Trajectory>
{
  return std::async(std::launch::async, [&frame, target, this]() { return plan(frame, target); });
}

std::future<PlannerPtr> make_planner(const mc_rbdyn::Robot & robot,
                                     const std::string & ef_body,
                                     const PlannerConfig & config)
{
  return std::async(std::launch::async,
                    [&robot, ef_body, config]() { return Planner::create(robot, ef_body, config); });
}

std::shared_ptr<mc_moveit::BSplineTrajectoryTask> Planner::Trajectory::setup_bspline_task(
    const mc_rbdyn::RobotFrame & frame,
    double stiffness,
    double weight) const
{
  if(error != 1)
  {
    mc_rtc::log::error("[mc_moveit] Cannot setup b-spline task as planning failed");
    return nullptr;
  }
  if(waypoints.size() <= 1)
  {
    mc_rtc::log::error("[mc_moveit] Not enough waypoints in the plan to setup a b-spline task");
    return nullptr;
  }
  mc_tasks::BSplineTrajectoryTask::waypoints_t posWp;
  posWp.reserve(waypoints.size() - 2);
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
  oriWp.reserve(waypoints.size() - 2);
  for(size_t i = 1; i < waypoints.size() - 1; ++i)
  {
    double time = waypoints[i].time;
    const auto & wp = waypoints[i].pose;
    posWp.push_back(wp.translation());
    oriWp.push_back({time, wp.rotation().normalized()});
  }
  const auto & last = waypoints.back();
  const auto & target = last.pose;
  const auto & duration = last.time;
  try
  {
    auto task =
        std::make_shared<mc_moveit::BSplineTrajectoryTask>(frame, duration, stiffness, weight, target, posWp, oriWp);
    task->reset();
    // // FIXME It seems that there is a bug in the rotation interpolation (it makes the end-effector oscillate)
    // // For now, reduce the stiffness in the central part of the trajectory to make it less visible
    Eigen::Vector6d dimStiff;
    dimStiff << 0.2, 0.2, 0.2, 1., 1., 1.;
    task->stiffnessInterpolation(
        {{0., stiffness * dimStiff}, {duration / 2, 0.1 * stiffness * dimStiff}, {duration, stiffness * dimStiff}});
    return task;
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[mc_moveit] Could not interpolate between waypoints: {}", e.what());
    return nullptr;
  }
}

std::shared_ptr<mc_moveit::PostureTrajectoryTask> Planner::Trajectory::setup_posture_task(
    const mc_solver::QPSolver & solver,
    mc_rbdyn::Robot & robot,
    double stiffness,
    double weight) const
{
  if(error != 1)
  {
    mc_rtc::log::error("[mc_moveit] Cannot setup b-spline task as planning failed");
    return nullptr;
  }
  if(waypoints.size() <= 1)
  {
    mc_rtc::log::error("[mc_moveit] Not enough waypoints in the plan to setup a posture trajectory task");
    return nullptr;
  }
  auto task = std::make_shared<mc_moveit::PostureTrajectoryTask>(solver, robot, postures);
  task->stiffness(stiffness);
  task->weight(weight);
  return task;
}

} // namespace mc_moveit
