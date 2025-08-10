#ifndef PS4_HPP
#define PS4_HPP

#include "ps_base.hpp"

namespace ps4
{
class ps : public ps_base
{
public:
  void get_data(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    cross_btn = msg->buttons[0];
    circle_btn = msg->buttons[1];
    triangle_btn = msg->buttons[3];
    square_btn = msg->buttons[2];

    L1_btn = msg->buttons[9];
    R1_btn = msg->buttons[10];
    // L2_btn = msg->buttons[18];
    // R2_btn = msg->buttons[19]; 
    L2_btn = !(L2>-0.5);
    R2_btn = !(R2>-0.5);

    select_btn = msg->buttons[4];
    share_btn = msg->buttons[4];
    create_btn = msg->buttons[4];

    start_btn = msg->buttons[6];
    options_btn = msg->buttons[6];

    PS_btn = msg->buttons[5];

    joy_left_x = msg->axes[0];
    joy_left_y = msg->axes[1];

    
    joy_right_x = msg->axes[2];
    joy_right_y = msg->axes[3];
    
    L2 = msg->axes[4];
    R2 = msg->axes[5];

    // d_pad_x = msg->axes[6];
    // d_pad_y = msg->axes[7];
    // d_pad2btn();

    // Change these indexes based on your "ros2 topic echo /joy" output
    up_btn    = msg->buttons[11];
    down_btn  = msg->buttons[12];
    left_btn  = msg->buttons[13];
    right_btn = msg->buttons[14];

    d_pad_x = left_btn-right_btn;
    d_pad_y = up_btn-down_btn;

  }


  // virtual void sub_joy_thread(const sensor_msgs::msg::Joy::SharedPtr msg)
  // {
  //     get_data(msg);
  // }
};
}

#endif
