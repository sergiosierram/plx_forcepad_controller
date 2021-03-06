#include <force_controller/spider_pad.h>

SpiderPad::SpiderPad():
    left_x_(1),
	left_y_(1),
	right_x_(1),
	right_y_(1)
  {
      current_frc = 1.0;

      //JOYSTICK PAD TYPE
      nh_.param<std::string>("pad_type",pad_type_,"ps3");
      //
      nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
      // MOTION CONF
      nh_.param("left_axis_x", left_x_, DEFAULT_LEFT_AXIS_X);
      nh_.param("left_axis_y", left_y_, DEFAULT_LEFT_AXIS_Y);
      nh_.param("right_axis_x", right_x_, DEFAULT_RIGHT_AXIS_X);
      nh_.param("right_axis_y", right_y_, DEFAULT_RIGHT_AXIS_Y);
      nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
	  nh_.param("force_topic_left", frc_topic_l_, frc_topic_l_);
	  nh_.param("force_topic_right", frc_topic_r_, frc_topic_r_);
      nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
      nh_.param<std::string>("joystick_dead_zone", joystick_dead_zone_, "true");

      ROS_INFO("ForcePad num_of_buttons_ = %d", num_of_buttons_);
      for(int i = 0; i < num_of_buttons_; i++){
          bRegisteredButtonEvent[i] = false;
          ROS_INFO("bREG %d", i);
          }

      for(int i = 0; i < 3; i++){
        bRegisteredDirectionalArrows[i] = false;
      }

      // Publish through the node handle Twist type messages to the guardian_controller/command topic
      frc_left_pub_ = nh_.advertise<geometry_msgs::Wrench>(frc_topic_l_, 1);
	  frc_right_pub_ = nh_.advertise<geometry_msgs::Wrench>(frc_topic_r_, 1);


      // Listen through the node handle sensor_msgs::Joy messages from joystick
      // (these are the references that we will sent to summit_xl_controller/command)
      pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SpiderPad::padCallback, this);

      // Diagnostics
      updater_pad.setHardwareID("None");
      // Topics freq control
      min_freq_command = min_freq_joy = 5.0;
      max_freq_command = max_freq_joy = 50.0;
      sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
                          diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

      pub_command_freq_l = new diagnostic_updater::HeaderlessTopicDiagnostic(frc_topic_l_.c_str(), updater_pad,
                          diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

	  pub_command_freq_r = new diagnostic_updater::HeaderlessTopicDiagnostic(frc_topic_r_.c_str(), updater_pad,
                          diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));


      bEnable = false;	// Communication flag disabled by default
      last_command_ = true;
  }


  void SpiderPad::Update(){
      updater_pad.update();
  }

  void SpiderPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
      geometry_msgs::Wrench frc_l;
      geometry_msgs::Wrench frc_r;

      frc_l.force.x = 0.0; frc_l.force.y = 0.0; frc_l.force.z = 0.0;
      frc_l.torque.x = 0.0; frc_l.torque.y = 0.0; frc_l.torque.z = 0.0;
	  
	  frc_r.force.x = 0.0; frc_r.force.y = 0.0; frc_r.force.z = 0.0;
      frc_r.torque.x = 0.0; frc_r.torque.y = 0.0; frc_r.torque.z = 0.0;

      bEnable = (joy->buttons[dead_man_button_] == 1);

      // Actions dependant on dead-man button
      if (joy->buttons[dead_man_button_] == 1) {

          frc_l.force.x = current_frc*l_scale_*joy->axes[left_x_];
          frc_l.force.y = current_frc*l_scale_*joy->axes[left_y_];
		  frc_l.force.z = 0;

          frc_r.force.x = current_frc*l_scale_*joy->axes[right_x_];
          frc_r.force.y = current_frc*l_scale_*joy->axes[right_y_];
		  frc_r.force.z = 0;
      }

      sus_joy_freq->tick();	// Ticks the reception of joy events

       // Publish
      // Only publishes if it's enabled
      if(bEnable){
          frc_left_pub_.publish(frc_l);
          pub_command_freq_l->tick();
		  frc_right_pub_.publish(frc_r);
          pub_command_freq_r->tick();
          last_command_ = true;
          }

      if(!bEnable && last_command_){

		  frc_l.force.x = 0.0; frc_l.force.y = 0.0; frc_l.force.z = 0.0;
          frc_l.torque.x = 0.0; frc_l.torque.y = 0.0; frc_l.torque.z = 0.0;
	  
	      frc_r.force.x = 0.0; frc_r.force.y = 0.0; frc_r.force.z = 0.0;
          frc_r.torque.x = 0.0; frc_r.torque.y = 0.0; frc_r.torque.z = 0.0;
          
          frc_left_pub_.publish(frc_l);
          pub_command_freq_l->tick();
		  frc_right_pub_.publish(frc_r);
          pub_command_freq_r->tick();
          last_command_ = false;
          }
  }


  int main(int argc, char** argv)
  {
      ros::init(argc, argv, "walker_pad");
      SpiderPad my_spider_pad;

      ros::Rate r(10.0);

      while( ros::ok() )
      {
          // UPDATING DIAGNOSTICS
          my_spider_pad.Update();
          ros::spinOnce();
          r.sleep();
      }
  }

