#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rbdyn/RobotFrame.h>

#include <mc_moveit/Planner.h>
#include <mc_tasks/PostureTask.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <vector>

struct MoveItPlan : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

    void make_plan();
    void make_posture_plan();
    
    // TODO move to another state
    void execute_plan(mc_control::fsm::Controller &controller);

    void add_obstacle(mc_control::fsm::Controller &ctl, const std::string &name, const Eigen::Vector3d & size);
    void update_obstacle_position(const std::string &name, const sva::PTransformd & pos);
    void remove_obstacle(mc_control::fsm::Controller &ctl, const std::string &name);

    void setup_interface(mc_control::fsm::Controller & ctl, const std::string & mode);

    void reset(mc_control::fsm::Controller &ctl);

    private:
        std::string end_effector_;
        mc_rbdyn::RobotFrame *frame_;

        bool trajectory_posture_execution;

        std::shared_ptr<mc_moveit::BSplineTrajectoryTask> ef_task = nullptr;
        std::shared_ptr<mc_moveit::PostureTrajectoryTask> posture_trajectory_task = nullptr;

        std::shared_ptr<mc_moveit::Planner> planner_;
        mc_moveit::Planner::Trajectory trajectory_;

        mc_tasks::PostureTask *posture_task_;

        std::map<std::string, std::vector<double>> posture_target_;

        sva::PTransformd target_;

        std::string status_;
};
