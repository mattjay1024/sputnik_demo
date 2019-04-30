#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <isc_joy/xinput.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/waypoint.h>

#include <functional>
#include <random>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movement_server;
typedef actionlib::SimpleClientGoalState Goals;

class random_goal_generator {
	public:
		random_goal_generator() {
			ros::NodeHandle nh_private("~");

			double max_forward_progress; // how much forward we can move, in meters
			double max_lateral_progress; // how far to the left or right we can move
			
			nh_private.param("max_forward_movement", max_forward_progress, 3.0);
			nh_private.param("max_lateral_movement", max_lateral_progress, 1.7);

			forward_dist = std::uniform_real_distribution<double>(0.1, max_forward_progress);
			lateral_dist = std::uniform_real_distribution<double>(-max_lateral_progress, max_lateral_progress);
		};

		geometry_msgs::Point32 generate_goal() {
			geometry_msgs::Point32 goal;
			
			goal.x = forward_dist(engine);
			goal.y = lateral_dist(engine);

			return goal;
		};

	private:
		std::uniform_real_distribution<double> forward_dist; // two distributions so we can have different bounds for each movement direction
		std::uniform_real_distribution<double> lateral_dist;
		std::mt19937 engine;
};

class sputnik_demo {
	public:
		sputnik_demo() : move_requests("move_base", true) {
			waypoint_requests = node.serviceClient<ohm_igvc_srvs::waypoint>("waypoint");
			
			ros::NodeHandle nh_private("~");
			
			manual_drive = nh_private.advertise<geometry_msgs::Twist>("cmd_vel", 1);
			joystick = nh_private.subscribe("joystick", 1, &sputnik_demo::joystick_update, this);

			move_requests.waitForServer(); // wait for the server connection to be established
			
			double wander_period;
			double route_follow_period;

			nh_private.param("wander_time", wander_period, 180.0);
			nh_private.param("route_follow_time", route_follow_period, 300.0);
			nh_private.param("enable_wandering", enable_wander, true);
			nh_private.param("enable_route", enable_route_follow, true);
			nh_private.param("odom_frame", odom_frame, std::string("odom"));
			nh_private.param("base_frame", base_frame, std::string("base_link"));

			wander_timer = node.createTimer(ros::Duration(wander_period), &sputnik_demo::wander_cb, this, true); // one-shot timers so we can control when they start
			route_timer = node.createTimer(ros::Duration(route_follow_period), &sputnik_demo::route_cb, this, true);
			state_printer = node.createTimer(ros::Duration(1.0), &sputnik_demo::print_state, this);
	
			wander_timer.stop(); // stop them so we can start them after first goal is submitted
			route_timer.stop();

			flip_FB = false;
			flip_LR = false;
			start_button_down = false;
			lock_forward_speed = false;

			auto_mode = false;
		};
		
		void wander_cb(const ros::TimerEvent &e) {
			if(enable_route_follow) { // if we are route following, reset timer and flags	
				wandering = false;
				route_timer_elapsed = false;
				route_timer.start();
			} // otherwise, this was called in error
		};

		void route_cb(const ros::TimerEvent &e) { 
			if(enable_wander) { // if we are wandering, reset timer and set flags
				wandering = true;
				route_timer_elapsed = true;
				wander_timer.start();
			} // otherwise, this was called in error
		};

		void joystick_update(const isc_joy::xinput::ConstPtr& joy) {
			if(joy->Start){ //The Start button has been pressed
				start_button_down = true;
				ROS_INFO("### START PRESSED ###");
			} else if(start_button_down && !joy->Start){ //The Start button has been released
				auto_mode = !auto_mode;
				if(!auto_mode) move_requests.cancelAllGoals();
				start_button_down = false;
				ROS_INFO("### START RELEASED ###");
			}

			if(!auto_mode){
        		bool enableDriving = joy->LB; //the dead man's switch

				//toggle flipping controls
				if(joy->Y && !enableDriving) flip_FB = !flip_FB;
				if(joy->X && !enableDriving) flip_LR = !flip_LR;

				double joySpeed = 0.0, joyTurn = 0.0;
				joySpeed = joy->LeftStick_UD * (flip_FB ? -1.0 : 1.0);
				joyTurn = joy->LeftStick_LR * (flip_LR ? -1.0 : 1.0);

				if(joy->A) {
					forward_speed = joySpeed;
					lock_forward_speed = true;
				} else if(joy->RB) {
					lock_forward_speed = false;
				}

				if(lock_forward_speed) joySpeed = forward_speed;

				geometry_msgs::Twist msg;
				msg.linear.x = enableDriving ? joySpeed : 0;
				msg.angular.z = enableDriving ? joyTurn : 0;
				manual_drive.publish(msg);

				// if(enableLogging) ROS_INFO("Manual Control: %s linear.x=%f angular.z=%f", joy->LB ? "on" : "off", msg.linear.x, msg.angular.z);
			} else {
				if(enable_route_follow) { 
					waypoint_id = 0;
					wandering = false;
					get_next_waypoint();
					route_timer.start();
				} else {
					wandering = true;
					goal = move_gen.generate_goal();
				}
			
				move_base_msgs::MoveBaseGoal goal_req;

				goal_req.target_pose.header.frame_id = (wandering ? base_frame : odom_frame); 
				goal_req.target_pose.header.stamp = ros::Time::now();
				goal_req.target_pose.pose.position.x = goal.x;
				goal_req.target_pose.pose.position.y = goal.y;
				goal_req.target_pose.pose.orientation.w = 1.0;

				move_requests.sendGoal(goal_req, boost::bind(&sputnik_demo::goal_finished, this, _1, _2));
			}
		};

		bool get_next_waypoint() {
			ohm_igvc_srvs::waypoint req_wp;

			req_wp.request.ID = waypoint_id;

			if(!waypoint_requests.call(req_wp)) return false; // probably means there are no more waypoints left

			goal.x = req_wp.response.waypoint.latitude;
			goal.y = req_wp.response.waypoint.longitude;

			waypoint_id++;

			return true;
		};

		void goal_finished(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result) {
			if(auto_mode) {
				if(wandering) { // if we're wandering, randomly generate another goal
					goal = move_gen.generate_goal();
				} else {
					if(!get_next_waypoint()) { // otherwise check for more waypoints
						if(route_timer_elapsed && enable_wander) { // in this case, we're out of time anyway so just switch to wandering
							goal = move_gen.generate_goal();
							wander_timer.start();
						} else { // otherwise, start another round of route following
							waypoint_id = 0;
							get_next_waypoint();
						}
					}
				}

				move_base_msgs::MoveBaseGoal goal_req;

				goal_req.target_pose.header.frame_id = (wandering ? base_frame : odom_frame);
				goal_req.target_pose.header.stamp = ros::Time::now();
				goal_req.target_pose.pose.position.x = goal.x;
				goal_req.target_pose.pose.position.y = goal.y;
				goal_req.target_pose.pose.orientation.w = 1.0;

				move_requests.sendGoal(goal_req, boost::bind(&sputnik_demo::goal_finished, this, _1, _2));
			}
		};

		void print_state(const ros::TimerEvent &e) {
			ROS_INFO("---\nauto_mode: %s\nwandering: %s\ngoal:\n\tx: %f\n\ty: %f\n\tz: %f\ngoal_state: %s\n", auto_mode ? "true" : "false", wandering ? "true" : "false", goal.x, goal.y, goal.z, move_requests.getState().toString().c_str());
		}

	private:
		ros::NodeHandle node;
		ros::Publisher manual_drive;
		ros::Subscriber joystick;
		ros::ServiceClient waypoint_requests;
		ros::Timer wander_timer, route_timer, state_printer;
		movement_server move_requests;

		random_goal_generator move_gen;
		
		bool enable_wander;
		bool enable_route_follow;
		bool wandering;
		bool route_timer_elapsed;
		bool has_goal;
		bool auto_mode;

		bool flip_FB;
		bool flip_LR;
		bool lock_forward_speed;
		bool start_button_down;

		int waypoint_id;

		double forward_speed;

		geometry_msgs::Point32 goal;
		
		std::string odom_frame;
		std::string base_frame;
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "sputnik_demo");
	
	sputnik_demo demo;

	ros::spin();

	return 0;
}
