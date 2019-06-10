#include "HROSCognitionMaraComponents.hpp"

#include <rttest/rttest.h>
#include <unistd.h>

int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rttest_set_sched_priority(48, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  char hostname[150];
  memset(hostname, 0, 150);
  if(gethostname(hostname, 150)==-1){
    return -2;
  }

  std::string node_name = "hros_cognition_" + std::string(hostname);
  std::replace(node_name.begin(), node_name.end(), '-', '_');

  std::cout << "node_name " << node_name << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<HROSCognitionMaraComponentsNode> lc_node = std::make_shared<HROSCognitionMaraComponentsNode>(node_name, argc, argv);

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
