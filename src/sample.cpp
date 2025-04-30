#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/RobotLoader.h>
#include <SpaceVecAlg/PTransform.h>

#include <mc_moveit/Planner.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string MainRobot = "Panda";
  std::string EndEffector = "";

  po::options_description desc("mc_moveit sample options");
  // clang-format off
  desc.add_options()
    ("help", "Show this help message")
    ("robot,r", po::value<std::string>(&MainRobot), "Robot used in this sample")
    ("end-effector,e", po::value<std::string>(&EndEffector), "End-effector used for planning");
  // clang-format on
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);
  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }

  mc_control::MCGlobalController::GlobalConfiguration config("", mc_rbdyn::RobotLoader::get_robot_module(MainRobot));
  config.include_halfsit_controller = false;
  config.enabled_controllers = {"Posture"};
  config.initial_controller = "Posture";

  // Start a dummy controller to get a published state
  mc_control::MCGlobalController controller(config);
  auto & robot = controller.robot();
  auto & frame = [&robot, &EndEffector]() -> mc_rbdyn::RobotFrame &
  {
    const std::string efFrameName = EndEffector.empty() ? "EndEffector" : EndEffector;
    if(!robot.hasFrame(efFrameName))
    {
      // FIXME: only works for Panda
      robot.makeFrame(efFrameName, robot.frame("panda_link7"), sva::PTransformd::Identity());
    }
    return robot.frame("EndEffector");
  }();

  std::vector<double> q;
  {
    auto & mbc = controller.robot().mbc();
    const auto & rjo = controller.ref_joint_order();
    for(const auto & jn : rjo)
    {
      if(controller.robot().hasJoint(jn))
      {
        for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)]) { q.push_back(qj); }
      }
      else
      {
        // FIXME This assumes that a joint that is in ref_joint_order but missing from the robot is of size 1 (very
        // likely to be true)
        q.push_back(0);
      }
    }
  }
  controller.setEncoderValues(q);
  controller.init(q);
  controller.running = true;
  // Start running
  auto now = std::chrono::steady_clock::now();
  auto dt = std::chrono::duration<double, std::micro>(1e6 * controller.timestep());
  auto & solver = controller.controller().solver();
  auto posture_task = dynamic_cast<mc_tasks::PostureTask *>(solver.tasks()[0]);
  posture_task->stiffness(10.0);
  std::shared_ptr<mc_moveit::BSplineTrajectoryTask> ef_task = nullptr;
  std::shared_ptr<mc_moveit::PostureTrajectoryTask> posture_trajectory_task = nullptr;
  bool trajectory_posture_execution = true;
  bool trajectory_executing = false;
  std::string str_state = "Waiting for plan request";

  auto clear_ef_task = [&]()
  {
    if(ef_task)
    {
      solver.removeTask(ef_task);
      ef_task.reset();
    }
    if(posture_trajectory_task)
    {
      solver.removeTask(posture_trajectory_task);
      posture_trajectory_task.reset();
    }
    solver.addTask(posture_task);
    trajectory_executing = false;
  };

  bool do_quick_reset = false;

  std::cout << "creating planner now" << std::endl;
  auto planner = mc_moveit::Planner::create(controller.robot(), frame.body());

  std::thread run_th(
      [&]()
      {
        while(controller.running && controller.run())
        {
          if(ef_task && ef_task->timeElapsed())
          {
            posture_task->reset();
            trajectory_executing = false;
          }
          if(posture_trajectory_task && posture_trajectory_task->timeElapsed())
          {
            posture_task->reset();
            trajectory_executing = false;
          }
          if(do_quick_reset)
          {
            do_quick_reset = false;
            clear_ef_task();
            posture_task->target(robot.module().stance());
            auto & mbc = controller.controller().robot().mbc();
            mbc.zero(robot.mb());
            for(const auto & [name, config] : robot.module().stance()) { mbc.q[robot.jointIndexByName(name)] =
            config; }
          }
          std::this_thread::sleep_until(now + dt);
          now = std::chrono::steady_clock::now();
        }
      });


  auto & gui = *controller.controller().gui();
  sva::PTransformd target = frame.position();
  std::map<std::string, std::vector<double>> posture_target = robot.module().stance();
  mc_moveit::Planner::Trajectory trajectory;
  auto make_plan = [&]()
  {
    if(trajectory_executing)
    {
      mc_rtc::log::error("Plan execution in progress");
      return;
    }
    trajectory = planner->plan(frame, target);
    if(trajectory.error == 1)
    {
      str_state = "Execution ready";
      mc_rtc::log::success("Execution ready");
    }
    else { str_state = "Planning failed"; }
  };
  auto make_posture_plan = [&]()
  {
    if(trajectory_executing)
    {
      mc_rtc::log::error("Plan execution in progress");
      return;
    }
    trajectory = planner->plan(frame, posture_target);
    if(trajectory.error == 1)
    {
      str_state = "Execution ready";
      mc_rtc::log::success("Execution ready");
    }
    else { str_state = "Planning failed"; }
  };
  auto execute_plan = [&]()
  {
    if(trajectory_executing)
    {
      mc_rtc::log::error("Plan is already executing");
      return;
    }
    if(trajectory.error != 1)
    {
      mc_rtc::log::error("No valid plan loaded");
      return;
    }
    if(trajectory.waypoints.size() <= 1)
    {
      mc_rtc::log::error("One point or less to play in the trajectory");
      return;
    }
    if(ef_task || posture_trajectory_task) { clear_ef_task(); }
    trajectory_executing = true;
    solver.removeTask(posture_task);
    if(trajectory_posture_execution)
    {
      posture_trajectory_task = trajectory.setup_posture_task(solver, robot, 1000.0, 1e5);
      solver.addTask(posture_trajectory_task);
    }
    else
    {
      ef_task = trajectory.setup_bspline_task(frame, 1000.0, 1e5);
      solver.addTask(ef_task);
    }
  };

  auto update_obstacle_position = [&](const std::string & name, const sva::PTransformd & pos)
  { planner->update_obstacle(name, pos); };
  auto remove_obstacle = [&](const std::string & name)
  {
    planner->remove_obstacle(name);
    gui.removeCategory({"MoveIt", "Collisions", name});
  };
  auto add_obstacle = [&](const std::string & name, const Eigen::Vector3d & size)
  {
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
    planner->add_obstacle(visual);
    gui.addElement(
        {"MoveIt", "Collisions", name}, mc_rtc::gui::Button("Remove", [&, name]() { remove_obstacle(name); }),
        mc_rtc::gui::Transform(
            "position", [&, name]() -> const sva::PTransformd & { return planner->get_obstacle(name).pose; },
            [&, name](const sva::PTransformd & p) { update_obstacle_position(name, p); }),
        mc_rtc::gui::Visual(
            "visual", [&, name]() -> const rbd::parsers::Visual & { return planner->get_obstacle(name).object; },
            [&, name]() -> const sva::PTransformd & { return planner->get_obstacle(name).pose; }));
    moveit_msgs::msg::CollisionObject object;
  };

  int fake_source = 0;
  std::function<void(const std::string &)> setup_interface;
  setup_interface = [&](const std::string & mode)
  {
    gui.removeElements(&fake_source);
    gui.addElement(&fake_source, {},
                   mc_rtc::gui::ComboInput(
                       "Planning mode", {"Target posture", "Target frame"}, [mode]() { return mode; },
                       [&setup_interface](const std::string & m) { setup_interface(m); }),
                   mc_rtc::gui::Label("State", [&]() -> const std::string & { return str_state; }),
                   mc_rtc::gui::Checkbox("Execute trajectory with posture", trajectory_posture_execution));
    if(mode == "Target frame")
    {
      gui.addElement(&fake_source, {},
                     mc_rtc::gui::Transform(
                         "Target", [&]() { return target; }, [&](const sva::PTransformd & t) { target = t; }),
                     mc_rtc::gui::Button("Plan", [&]() { make_plan(); }));
    }
    else if(mode == "Target posture")
    {
      gui.addElement(&fake_source, {},
                     mc_rtc::gui::Button("Set posture target",
                                         [&]()
                                         {
                                           for(size_t i = 0; i < robot.mb().joints().size(); ++i)
                                           {
                                             const auto & j = robot.mb().joint(i);
                                             if(j.dof() == 1) { posture_target[j.name()] = robot.mbc().q[i]; }
                                           }
                                         }),
                     mc_rtc::gui::Button("Plan", [&]() { make_posture_plan(); }));
    }
    gui.addElement(&fake_source, {}, mc_rtc::gui::Button("Execute", [&]() { execute_plan(); }),
                   mc_rtc::gui::Button("Clear executing task", [&]() { clear_ef_task(); }),
                   mc_rtc::gui::Button("Clear executing task & reset robot",
                                       [&]()
                                       {
                                         clear_ef_task();
                                         posture_task->target(robot.module().stance());
                                       }),
                   mc_rtc::gui::Button("Quick reset", [&]() { do_quick_reset = true; }),
                   mc_rtc::gui::Form(
                       "Add obstacle",
                       [&](const mc_rtc::Configuration & config) { add_obstacle(config("Name"), config("Size")); },
                       mc_rtc::gui::FormStringInput("Name", true, "Box"),
                       mc_rtc::gui::FormArrayInput("Size", true, Eigen::Vector3d{0.1, 0.1, 0.1})),
                   mc_rtc::gui::Button("Exit", [&]() { controller.running = false; }));
  };
  setup_interface("Target posture");


  if(run_th.joinable()) { run_th.join(); }
  rclcpp::shutdown();
  return 0;
}
