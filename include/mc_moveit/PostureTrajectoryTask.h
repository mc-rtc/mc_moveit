#include <mc_moveit/api.h>

#include <mc_tasks/PostureTask.h>

namespace mc_moveit
{

struct MC_MOVEIT_DLLAPI TimedPosture
{
  double time;
  std::map<std::string, std::vector<double>> posture;
};

struct MC_MOVEIT_DLLAPI PostureTrajectoryTask : public mc_tasks::PostureTask
{
  PostureTrajectoryTask(const mc_solver::QPSolver &,
                        mc_rbdyn::Robot & robot,
                        const std::vector<TimedPosture> & postures);

  void addToSolver(mc_solver::QPSolver &) override;

  void update(mc_solver::QPSolver &) override;

  inline bool timeElapsed() const noexcept
  {
    return time_elapsed_;
  }

private:
  double t_ = 0.0;
  size_t postures_idx_ = 0;
  bool time_elapsed_ = false;
  std::vector<TimedPosture> postures_;
  std::map<std::string, std::vector<double>> target_;
  std::map<std::string, double> posture_increment_;
};

} // namespace mc_moveit
