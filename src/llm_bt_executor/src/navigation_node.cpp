#include <behaviortree_cpp/bt_factory.h>

#include "navigation_node.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<NavigationAction>("NavigationAction");
}

BT::NodeStatus NavigationAction::onStart()
{
  std::cout << "Task Begin!";

  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (220 ms)
  _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(220);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigationAction::onRunning()
{
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Pretend that, after a certain amount of time,
  // we have completed the operation
  if(std::chrono::system_clock::now() >= _completion_time)
  {
    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigationAction::onHalted()
{
  printf("[ MoveBase: ABORTED ]");
}
