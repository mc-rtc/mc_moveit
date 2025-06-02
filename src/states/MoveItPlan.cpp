#include "MoveItPlan.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>

#include "../MoveItFSM.h"
#include "mc_moveit/Planner.h"

void MoveItPlan::configure(const mc_rtc::Configuration & config)
{
    config("end_effector", end_effector_);

    config_.load(config);
}

void MoveItPlan::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MoveItFSM &>(ctl_);

  status_ = "Waiting request";

  std::string efFrameName = end_effector_.empty() ? "EndEffector" : end_effector_;
  if(!ctl.robot().hasFrame(efFrameName)){
    frame_ = &ctl.robot().makeFrame(efFrameName, ctl.robot().frame(end_effector_), sva::PTransformd::Identity());
  }
  else{
    frame_ = &ctl.robot().frame(efFrameName);
  }

  target_ = frame_->position();

  planner_ = mc_moveit::Planner::create(ctl.robot(), frame_->body());

  setup_interface(ctl, "Target posture");

  posture_task_ = dynamic_cast<mc_tasks::PostureTask *>(ctl.solver().tasks()[0]);
  posture_task_->stiffness(10.0);
}

bool MoveItPlan::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MoveItFSM &>(ctl_);

  if(ef_task && ef_task->timeElapsed())
  {
    posture_task_->reset();
    status_ = "Waiting request";
  }
  if(posture_trajectory_task && posture_trajectory_task->timeElapsed())
  {
    posture_task_->reset();
    status_ = "Waiting request";
  }
  return false;
}

void MoveItPlan::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MoveItFSM &>(ctl_);

  ctl.solver().removeTask(ef_task);
  ef_task.reset();
  ctl.solver().removeTask(posture_trajectory_task);
  posture_trajectory_task.reset();
}

void MoveItPlan::make_plan(){
    if(status_ == "Executing plan")
    {
        mc_rtc::log::error("Plan execution in progress");
        return;
    }

    trajectory_ = planner_->plan(*frame_, target_);

    if(trajectory_.error == 1)
    {
        status_ = "Execution ready";
        mc_rtc::log::success("Execution ready");
    }
    else { 
      status_ = "Planning failed"; 
      mc_rtc::log::warning("Planning failed");
    }
}

void MoveItPlan::make_posture_plan(){
  if(status_ == "Executing plan"){
    mc_rtc::log::error("Plan is already executing");
    return;
  }
  trajectory_ = planner_->plan(*frame_, posture_target_);
  if(trajectory_.error == 1)
    {
      status_ = "Execution ready";
      mc_rtc::log::success("Execution ready");
    }
  else { status_ = "Planning failed"; }
}

void MoveItPlan::execute_plan(mc_control::fsm::Controller & controller){
  if(status_ == "Executing plan"){
    mc_rtc::log::error("Plan is already executing");
    return;
  }
  if(trajectory_.error != 1){
    mc_rtc::log::error("No valid plan loaded");
    return;
  }
  if(trajectory_.waypoints.size() <= 1){
    mc_rtc::log::error("One point or less to play in the trajectory");
    return;
  }

  if(ef_task || posture_trajectory_task) {reset(controller);}
  status_ = "Executing plan";
  controller.solver().removeTask(posture_task_);
  if(trajectory_posture_execution){
    posture_trajectory_task = trajectory_.setup_posture_task(controller.solver(), controller.robot(), 1000, 1e5);
    controller.solver().addTask(posture_trajectory_task);
  }
  else{
    ef_task = trajectory_.setup_bspline_task(*frame_, 1000, 1e5);
    controller.solver().addTask(ef_task);
  }

}

void MoveItPlan::setup_interface(mc_control::fsm::Controller & ctl, const std::string & mode){
    ctl.gui()->removeCategory({"MoveIt"});
    ctl.gui()->addElement({"MoveIt"}, {},
                   mc_rtc::gui::ComboInput(
                       "Planning mode", {"Target posture", "Target frame"}, [mode]() { return mode; },
                       [&](const std::string & m) { setup_interface(ctl, m); }),
                   mc_rtc::gui::Label("State", [&]() -> const std::string & { 
                        return status_;
                    }),
                   mc_rtc::gui::Checkbox("Execute trajectory with posture", trajectory_posture_execution));
    if(mode == "Target frame")
    {
      ctl.gui()->addElement({"MoveIt"}, {},
                     mc_rtc::gui::Transform(
                         "Target", [&]() { return target_; }, [&](const sva::PTransformd & t) { target_ = t; }),
                     mc_rtc::gui::Button("Plan", [&]() { make_plan(); }));
    }
    else if(mode == "Target posture")
    {
      ctl.gui()->addElement({"MoveIt"}, {},
                     mc_rtc::gui::Button("Set posture target",
                                         [&]()
                                         {
                                           for(size_t i = 0; i < ctl.robot().mb().joints().size(); ++i)
                                           {
                                             const auto & j = ctl.robot().mb().joint(i);
                                             if(j.dof() == 1) { posture_target_[j.name()] = ctl.robot().mbc().q[i]; }
                                           }
                                         }),
                     mc_rtc::gui::Button("Plan", [&]() { make_posture_plan(); }));
    }
    ctl.gui()->addElement({"MoveIt"}, {}, 
      mc_rtc::gui::Button("Execute", [&]() { execute_plan(ctl); }),
      mc_rtc::gui::Button("Clear executing task", [&]() { reset(ctl); }),
      mc_rtc::gui::Button("Clear executing task & reset robot",
                          [&]()
                          {
                            reset(ctl);
                            posture_task_->target(ctl.robot().module().stance());
                          }),
      mc_rtc::gui::Form("Add obstacle",
                       [&](const mc_rtc::Configuration & config) { add_obstacle(ctl, config("Name"), config("Size")); },
                       mc_rtc::gui::FormStringInput("Name", true, "Box"),
                       mc_rtc::gui::FormArrayInput("Size", true, Eigen::Vector3d{0.1, 0.1, 0.1})));
}

void MoveItPlan::reset(mc_control::fsm::Controller &ctl){
  if(ef_task){
    ctl.solver().removeTask(ef_task);
    ef_task->reset();
  }
  if(posture_trajectory_task){
    ctl.solver().removeTask(posture_trajectory_task);
    posture_trajectory_task->reset();
  }

  ctl.solver().addTask(posture_task_);
  status_ = "Waiting request";
}

void MoveItPlan::add_obstacle(mc_control::fsm::Controller &ctl, const std::string &name, const Eigen::Vector3d &size){
  rbd::parsers::Visual visual;
    visual.name = name;
    visual.origin = sva::PTransformd::Identity();
    visual.geometry.type = rbd::parsers::Geometry::Type::BOX;
    auto box = rbd::parsers::Geometry::Box{};
    box.size = size;
    visual.geometry.data = box;
    visual.material.type = rbd::parsers::Material::Type::COLOR;
    auto color = rbd::parsers::Material::Color{};
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 0.7;
    visual.material.data = color;
    planner_->add_obstacle(visual);
    ctl.gui()->addElement(
        {"MoveIt", "Collisions", name}, mc_rtc::gui::Button("Remove", [&, name]() { remove_obstacle(ctl, name); }),
        mc_rtc::gui::Transform(
            "position", [&, name]() -> const sva::PTransformd & { return planner_->get_obstacle(name).pose; },
            [&, name](const sva::PTransformd & p) { update_obstacle_position(name, p); }),
        mc_rtc::gui::Visual(
            "visual", [&, name]() -> const rbd::parsers::Visual & { return planner_->get_obstacle(name).object; },
            [&, name]() -> const sva::PTransformd & { return planner_->get_obstacle(name).pose; }));
    moveit_msgs::msg::CollisionObject object;
}

void MoveItPlan::update_obstacle_position(const std::string &name, const sva::PTransformd &pos){
  planner_->update_obstacle(name, pos);
}

void MoveItPlan::remove_obstacle(mc_control::fsm::Controller &ctl, const std::string &name){
  planner_->remove_obstacle(name);
  ctl.gui()->removeCategory({"MoveIt", "Collisions", name});
}

EXPORT_SINGLE_STATE("MoveItPlan", MoveItPlan)
