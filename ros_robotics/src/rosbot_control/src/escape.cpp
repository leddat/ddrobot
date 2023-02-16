#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>

#include <math.h>
#include <string>

using namespace std;

class RosbotMove {
public:
  RosbotClass rosbot;
  void get_out();
  float calc_distance(float x_0, float y_0, float x_1, float y_1);
};

void RosbotMove::get_out() {
  rosbot.move_forward(1);
  while (rosbot.get_laser(0) > 1.75) {
    ROS_INFO_STREAM("Laser frontal reading: " << rosbot.get_laser(0));
    rosbot.move_forward(1);
  }
  rosbot.turn("clockwise", 3);
  rosbot.move_forward(2);
  rosbot.turn("counterclockwise", 3);

  // Get initial position
  float x_0 = rosbot.get_position(1);
  float y_0 = rosbot.get_position(2);
  float x_1 = x_0;
  float y_1 = y_0;
  float dist = calc_distance(x_0, y_0, x_1, y_1);
  while (dist < 8.00) {
    // Update current position
    x_1 = rosbot.get_position(1);
    y_1 = rosbot.get_position(2);
    dist = calc_distance(x_0, y_0, x_1, y_1);
    ROS_INFO_STREAM("Distance travelled: " << dist);
    // Keep moving
    rosbot.move_forward(1);
  }
  rosbot.turn("clockwise", 3);
  rosbot.move_forward(5);
  ROS_INFO_STREAM("Success!!!");
}

float RosbotMove::calc_distance(float x_0, float y_0, float x_1, float y_1) {
  return sqrt(pow((x_0 - x_1), 2) + pow((x_0 - x_1), 2));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Rosbot_move_node");
  RosbotMove rosbot_moves;
  rosbot_moves.get_out();
}