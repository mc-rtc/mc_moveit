#include <mc_moveit/PostureTrajectoryTask.h>

namespace mc_moveit
{

PostureTrajectoryTask::PostureTrajectoryTask(const mc_solver::QPSolver & solver,
                                             mc_rbdyn::Robot & robot,
                                             const std::vector<TimedPosture> & postures)
: mc_tasks::PostureTask(solver, robot.robotIndex()), postures_(postures)
{
  type_ = "posture_trajectory";
  name_ = fmt::format("posture_trajectory_{}", robot.name());
}

void PostureTrajectoryTask::addToSolver(mc_solver::QPSolver & solver)
{
  mc_tasks::PostureTask::addToSolver(solver);
  t_ = 0.0;
  postures_idx_ = 0;
  target_ = postures_[0].posture;
  for(const auto & [name, target] : postures_[0].posture) { posture_increment_[name] = 0.0; }
  target(target_);
}

void PostureTrajectoryTask::update(mc_solver::QPSolver & solver)
{
  mc_tasks::PostureTask::update(solver);
  if(time_elapsed_) { return; }
  if(t_ >= postures_[postures_idx_].time)
  {
    // Number of control steps until the next posture
    size_t nsteps = 0;
    while(nsteps == 0 && postures_idx_ < postures_.size() - 1)
    {
      nsteps = std::floor((postures_[postures_idx_ + 1].time - postures_[postures_idx_].time) / solver.dt());
      postures_idx_++;
    }
    if(nsteps != 0)
    {
      auto & next_posture = postures_[postures_idx_].posture;
      for(const auto & [name, target] : next_posture)
      {
        posture_increment_[name] = (next_posture[name][0] - target_[name][0]) / nsteps;
      }
    }
    if(postures_idx_ == postures_.size() - 1)
    {
      target(postures_.back().posture);
      time_elapsed_ = true;
      return;
    }
  }
  for(const auto & [name, increment] : posture_increment_) { target_[name][0] += increment; }
  target(target_);
  t_ += solver.dt();
}

} // namespace mc_moveit
