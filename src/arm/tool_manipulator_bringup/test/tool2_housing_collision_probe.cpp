// Offline test tool2 collision probe. Uses the unchanged SRDF collision matrix.
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>
#include <sstream>

std::string read_file(const char* path)
{
  std::ifstream stream(path);
  return {std::istreambuf_iterator<char>(stream), {}};
}

int main(int argc, char** argv)
{
  if (argc != 3)
    return 2;
  auto urdf = urdf::parseURDF(read_file(argv[1]));
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, read_file(argv[2])))
    return 3;
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  if (model->getLinkModel("wrist_link_yaw")->getShapes().empty())
    return 4;
  planning_scene::PlanningScene scene(model);
  auto& state = scene.getCurrentStateNonConst();
  state.setToDefaultValues();
  const char* names[] = {"base_joint", "shoulder_joint", "elbow_joint",
                         "wrist_pitch_joint", "wrist_roll_joint", "wrist_yaw_joint", "ee_joint"};
  std::string line;
  while (std::getline(std::cin, line))
  {
    std::istringstream values(line);
    for (const auto* name : names)
    {
      double position;
      if (!(values >> position))
        return 5;
      state.setJointPositions(name, &position);
    }
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 100;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    std::cout << (!result.collision && state.satisfiesBounds()) << " ";
    for (const auto& contact : result.contacts)
    {
      const auto& detail = contact.second.front();
      std::cout << contact.first.first << ":" << contact.first.second << ":"
                << detail.pos.x() << ":" << detail.pos.y() << ":" << detail.pos.z()
                << ":" << detail.depth << ",";
    }
    std::cout << std::endl;
  }
}
