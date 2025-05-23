#include <sys/ioctl.h>
#include "linux/i2c-dev.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "rosrider_interfaces/msg/diagnostics.hpp"

#include <tf2_ros/transform_broadcaster.h>

#ifdef __cplusplus
extern "C"{
#endif

#include "i2c_hal.h"
#include "rosrider.h"

#ifdef __cplusplus
}
#endif

#define MAX_I2C_READ_FAIL 16
#define MAX_I2C_CHECKSUM_FAIL 16

using namespace std;
using std::placeholders::_1;

const std::string i2c_filename = "/dev/i2c-1";

// TODO: detect node if node already running and exit if so
// TODO: exit(0) does not trigger on_shutdown, if exiting due to threshold need to trigger shutdown
// TODO: driver can detect state of read, was it checksum, seq repeat, seq jump, depending on that state, next cycle calculate for two steps.
// TODO: base_width default 0.1, equalize on firmare

class ROSRider : public rclcpp::Node {

	public:

		ROSRider() : Node("rosrider_node") {

		    rclcpp::on_shutdown([this]() {
		        send_sysctl(fd, 0x05);                                      // system hibernate
                if(ros2rpi_config > 0) { send_hat_command(fd, 0x0); }       // hat off runs only if ros2rpi_config > 0
            });

            // uint8 parameters
			this->declare_parameter("CONFIG_FLAGS", DEFAULT_CONFIG_FLAGS);
			this->declare_parameter("UPDATE_RATE", DEFAULT_UPDATE_RATE);
			this->declare_parameter("PWM_DIV", DEFAULT_PWM_DIV);
			this->declare_parameter("DRIVE_MODE", DEFAULT_DRIVE_MODE);
			this->declare_parameter("MONITOR_RATE", DEFAULT_MONITOR_RATE);
			this->declare_parameter("ALLOWED_SKIP", DEFAULT_ALLOWED_SKIP);
			this->declare_parameter("I2C_ADDRESS", DEFAULT_I2C_ADDRESS);

			params_uint8[PARAM_CONFIG_FLAGS] = this->get_parameter("CONFIG_FLAGS").as_int();
			params_uint8[PARAM_UPDATE_RATE] = this->get_parameter("UPDATE_RATE").as_int();
			params_uint8[PARAM_PWM_DIV] = this->get_parameter("PWM_DIV").as_int();
			params_uint8[PARAM_DRIVE_MODE] = this->get_parameter("DRIVE_MODE").as_int();
			params_uint8[PARAM_MONITOR_RATE] = this->get_parameter("MONITOR_RATE").as_int();
			params_uint8[PARAM_ALLOWED_SKIP] = this->get_parameter("ALLOWED_SKIP").as_int();
			params_uint8[PARAM_I2C_ADDRESS] = this->get_parameter("I2C_ADDRESS").as_int();

            // uint16 parameters
			this->declare_parameter("PWM_SCALE", DEFAULT_PWM_SCALE);
			this->declare_parameter("PWM_FRQ", DEFAULT_PWM_FRQ);
			this->declare_parameter("MAX_IDLE_SECONDS", DEFAULT_MAX_IDLE_SECONDS);
			this->declare_parameter("UPPER_LIMIT", DEFAULT_UPPER_LIMIT);
			this->declare_parameter("INTEGRAL_LIMIT", DEFAULT_INTEGRAL_LIMIT);
			this->declare_parameter("ENCODER_PPR", DEFAULT_ENCODER_PPR);

			params_uint16[PARAM_PWM_SCALE] = this->get_parameter("PWM_SCALE").as_int();
			params_uint16[PARAM_PWM_FRQ] = this->get_parameter("PWM_FRQ").as_int();
			params_uint16[PARAM_MAX_IDLE_SECONDS] = this->get_parameter("MAX_IDLE_SECONDS").as_int();
			params_uint16[PARAM_UPPER_LIMIT] = this->get_parameter("UPPER_LIMIT").as_int();
			params_uint16[PARAM_INTEGRAL_LIMIT] = this->get_parameter("INTEGRAL_LIMIT").as_int();
			params_uint16[PARAM_ENCODER_PPR] = this->get_parameter("ENCODER_PPR").as_int();

            // uint32 parameters
			this->declare_parameter("RTC_TRIM", DEFAULT_RTC_TRIM);
			params_uint32[PARAM_RTC_TRIM] = this->get_parameter("RTC_TRIM").as_int();

            // int16 parameters
			this->declare_parameter("LEFT_FORWARD_DEADZONE", DEFAULT_LEFT_FORWARD_DEADZONE);
			this->declare_parameter("LEFT_REVERSE_DEADZONE", DEFAULT_LEFT_REVERSE_DEADZONE);
			this->declare_parameter("RIGHT_FORWARD_DEADZONE", DEFAULT_RIGHT_FORWARD_DEADZONE);
			this->declare_parameter("RIGHT_REVERSE_DEADZONE", DEFAULT_RIGHT_REVERSE_DEADZONE);

			params_int16[PARAM_LEFT_FORWARD_DEADZONE] = this->get_parameter("LEFT_FORWARD_DEADZONE").as_int();
			params_int16[PARAM_LEFT_REVERSE_DEADZONE] = this->get_parameter("LEFT_REVERSE_DEADZONE").as_int();
			params_int16[PARAM_RIGHT_FORWARD_DEADZONE] = this->get_parameter("RIGHT_FORWARD_DEADZONE").as_int();
			params_int16[PARAM_RIGHT_REVERSE_DEADZONE] = this->get_parameter("RIGHT_REVERSE_DEADZONE").as_int();

			// boolean parameters
			this->declare_parameter("AUTO_SYNC", DEFAULT_AUTO_SYNC);
			params_bool[PARAM_AUTO_SYNC] = this->get_parameter("AUTO_SYNC").as_bool();

            // float parameters
			this->declare_parameter("GEAR_RATIO", DEFAULT_GEAR_RATIO);
			this->declare_parameter("WHEEL_DIA", DEFAULT_WHEEL_DIA);
			this->declare_parameter("BASE_WIDTH", DEFAULT_BASE_WIDTH);
			this->declare_parameter("MAIN_AMP_LMT", DEFAULT_MAIN_AMP_LIMIT);
			this->declare_parameter("BAT_VOLTS_HIGH", DEFAULT_BAT_VOLTS_HIGH);
			this->declare_parameter("BAT_VOLTS_LOW", DEFAULT_BAT_VOLTS_LOW);
			this->declare_parameter("MAX_RPM", DEFAULT_MAX_RPM);
			this->declare_parameter("LEFT_AMP_LMT", DEFAULT_LEFT_AMP_LIMIT);
			this->declare_parameter("RIGHT_AMP_LMT", DEFAULT_RIGHT_AMP_LIMIT);

			this->declare_parameter("LEFT_KP", DEFAULT_LEFT_KP);
			this->declare_parameter("LEFT_KI", DEFAULT_LEFT_KI);
			this->declare_parameter("LEFT_KD", DEFAULT_LEFT_KD);
			this->declare_parameter("RIGHT_KP", DEFAULT_RIGHT_KP);
			this->declare_parameter("RIGHT_KI", DEFAULT_RIGHT_KI);
			this->declare_parameter("RIGHT_KD", DEFAULT_RIGHT_KD);

			this->declare_parameter("GAIN", DEFAULT_GAIN);
			this->declare_parameter("TRIM", DEFAULT_TRIM);
			this->declare_parameter("MOTOR_CONSTANT", DEFAULT_MOTOR_CONSTANT);

			params_float[PARAM_GEAR_RATIO] = (float) this->get_parameter("GEAR_RATIO").as_double();
			params_float[PARAM_WHEEL_DIA] = (float) this->get_parameter("WHEEL_DIA").as_double();
			params_float[PARAM_BASE_WIDTH] = (float) this->get_parameter("BASE_WIDTH").as_double();
			params_float[PARAM_MAIN_AMP_LIMIT] = (float) this->get_parameter("MAIN_AMP_LMT").as_double();
			params_float[PARAM_BAT_VOLTS_HIGH] = (float) this->get_parameter("BAT_VOLTS_HIGH").as_double();
			params_float[PARAM_BAT_VOLTS_LOW] = (float) this->get_parameter("BAT_VOLTS_LOW").as_double();
			params_float[PARAM_MAX_RPM] = (float) this->get_parameter("MAX_RPM").as_double();
			params_float[PARAM_LEFT_AMP_LIMIT] = (float) this->get_parameter("LEFT_AMP_LMT").as_double();
			params_float[PARAM_RIGHT_AMP_LIMIT] = (float) this->get_parameter("RIGHT_AMP_LMT").as_double();
			
			params_float[PARAM_LEFT_KP] = (float) this->get_parameter("LEFT_KP").as_double();
			params_float[PARAM_LEFT_KI] = (float) this->get_parameter("LEFT_KI").as_double();
			params_float[PARAM_LEFT_KD] = (float) this->get_parameter("LEFT_KD").as_double();
			params_float[PARAM_RIGHT_KP] = (float) this->get_parameter("RIGHT_KP").as_double();
			params_float[PARAM_RIGHT_KI] = (float) this->get_parameter("RIGHT_KI").as_double();
			params_float[PARAM_RIGHT_KD] = (float) this->get_parameter("RIGHT_KD").as_double();

			params_float[PARAM_GAIN] = (float) this->get_parameter("GAIN").as_double();
			params_float[PARAM_TRIM] = (float) this->get_parameter("TRIM").as_double();
			params_float[PARAM_MOTOR_CONSTANT] = (float) this->get_parameter("MOTOR_CONSTANT").as_double();
			
			// local parameters
			this->declare_parameter("I2C_ENABLED", true);
		    this->declare_parameter("ODOM_FRAME_ID", "odom");
		    this->declare_parameter("BASE_FRAME_ID", "base_footprint");	  
		    this->declare_parameter("BROADCAST_TF2", true);
		    this->declare_parameter("PUB_ODOMETRY", true);
		    this->declare_parameter("PUB_JOINTS", true); 
		    this->declare_parameter("PUB_DIAGNOSTICS", true); 
		    this->declare_parameter("DEBUG", false);
		    this->declare_parameter("ROS2RPI_CONFIG", 0x0);

		    // cached parameters
		    i2c_enabled = this->get_parameter("I2C_ENABLED").as_bool();
		    odom_frame_id = this->get_parameter("ODOM_FRAME_ID").as_string();
	        base_frame_id = this->get_parameter("BASE_FRAME_ID").as_string();
		    broadcast_tf2 = this->get_parameter("BROADCAST_TF2").as_bool();
		    pub_odometry = this->get_parameter("PUB_ODOMETRY").as_bool();
		    pub_joints = this->get_parameter("PUB_JOINTS").as_bool(); 
		    pub_diagnostics = this->get_parameter("PUB_DIAGNOSTICS").as_bool();  
		    debug_mode = this->get_parameter("DEBUG").as_bool();
		    ros2rpi_config = this->get_parameter("ROS2RPI_CONFIG").as_int();
			
			// i2c-init
			if(i2c_enabled) {

				if((fd = open(i2c_filename.c_str(), O_RDWR)) < 0) {
			    	RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus. Error code: %d", fd);
			        exit(0);
			    }

			    if(ioctl(fd, I2C_TIMEOUT, 3) < 0) {
					RCLCPP_ERROR(this->get_logger(), "Failed to set I2C_TIMEOUT");
			    }

			    if(ioctl(fd, I2C_RETRIES, 2) < 0) {
					RCLCPP_ERROR(this->get_logger(), "Failed to set I2C_RETRIES");
			    }

                // turn on hat with: PSEL_3V3_A, PSEL_3V3_B, PSEL_LIDAR, LIDAR_TX_ON
                if(ros2rpi_config > 0) { send_hat_command(fd, ros2rpi_config); }

                // allow board to power up
		        rclcpp::sleep_for(1000ms);

			    if((rv = ioctl(fd, I2C_SLAVE, params_uint8[PARAM_I2C_ADDRESS])) < 0) {
			    	RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to device. Error code: %d", fd);
			        exit(0);
			    }

			    rv = read_status(fd);
			    if(rv != 0) {
			    	RCLCPP_ERROR(this->get_logger(), "Initial READ_STATUS Failed. Error code: %s", strerror(rv));
			    	exit(0);
			    }

				// send params to device
			    uint32_t param_fail_count = send_parameters(fd);		
			    if(param_fail_count > 0) {
			    	RCLCPP_INFO(this->get_logger(), "Parameter Error Count: %d", param_fail_count);
			    	exit(0);
			    }

			    send_device_reset();
			    rclcpp::sleep_for(3000ms);

			}

			// notice: calculated and cached params must take place
			// after parameters are sent

			// calculated parameters, according to parameters from yaml
			PULSE_PER_REV = params_float[PARAM_GEAR_RATIO] * params_uint16[PARAM_ENCODER_PPR];
		    WHEEL_CIRCUMFERENCE = params_float[PARAM_WHEEL_DIA] * MATHPI;
		    TICKS_PER_METER = PULSE_PER_REV * (1.0f / WHEEL_CIRCUMFERENCE);
		    ROUNDS_PER_MINUTE = (60.0f / (1.0f / params_uint8[PARAM_UPDATE_RATE])) / PULSE_PER_REV;
		    LINEAR_RPM = (1.0f / WHEEL_CIRCUMFERENCE) * 60.0f;
		    ANGULAR_RPM = (params_float[PARAM_BASE_WIDTH] / (WHEEL_CIRCUMFERENCE * 2.0f)) * 60.0f;
		    COMMAND_TIMEOUT_SECS = ((double) params_uint8[PARAM_ALLOWED_SKIP]) / params_uint8[PARAM_UPDATE_RATE];
		    UPDATE_PERIOD = 1.0 / params_uint8[PARAM_UPDATE_RATE];
		    MONITOR_PERIOD = 1.0 / params_uint8[PARAM_MONITOR_RATE];
		    
		    // cached parameters for diff drive
		    MAX_RPM = params_float[PARAM_MAX_RPM];
			GAIN = params_float[PARAM_GAIN];
			TRIM = params_float[PARAM_TRIM];
			MOTOR_CONSTANT = params_float[PARAM_MOTOR_CONSTANT];
			
			// calculate trim 
		    MOTOR_CONSTANT_LEFT = (GAIN + TRIM) / MOTOR_CONSTANT;
		    MOTOR_CONSTANT_RIGHT = (GAIN - TRIM) / MOTOR_CONSTANT;

		    // calculate boolean parameters for display
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b00000001) { LEFT_REVERSE = true; } else { LEFT_REVERSE = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b00000010) { RIGHT_REVERSE = true; } else { RIGHT_REVERSE = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b00000100) { LEFT_SWAP = true; } else { LEFT_SWAP = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b00001000) { RIGHT_SWAP = true; } else { RIGHT_SWAP = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b00010000) { LEFT_ENC_AB = true; } else { LEFT_ENC_AB = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b00100000) { RIGHT_ENC_AB = true; } else { RIGHT_ENC_AB = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b01000000) { MODE1 = true; } else { MODE1 = false; }
    		if(params_uint8[PARAM_CONFIG_FLAGS] & 0b10000000) { MODE2 = true; } else { MODE2 = false; }

			// publishers
			if(pub_odometry) {
				odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(QOS_HIST_DEPTH));
      		}

      		if(broadcast_tf2) {
      			tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      		}

      		if(pub_joints) {
				joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(QOS_HIST_DEPTH));
      		}

      		if(pub_diagnostics) {
				diag_pub = this->create_publisher<rosrider_interfaces::msg::Diagnostics>("/rosrider/diagnostics", rclcpp::QoS(QOS_HIST_DEPTH));
			}

      		// cmd_vel subscriber
      		cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1), std::bind(&ROSRider::cmd_callback, this, _1));

			// main timer
			timer_i2c = this->create_wall_timer(1000ms / params_uint8[PARAM_UPDATE_RATE], std::bind(&ROSRider::timer_i2c_callback, this));

		    auto param_change_callback =

		      [this](std::vector<rclcpp::Parameter> parameters)
		      {
		        auto result = rcl_interfaces::msg::SetParametersResult();
		        result.successful = true;

		        for(auto parameter : parameters) {

		          rclcpp::ParameterType parameter_type = parameter.get_type();

		          if(rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
		          	result.successful = false;
		          } else {

		          	uint8_t pidtune_result = 0;
		          	if(parameter.get_name()=="LEFT_KP") {
					    pidtune_result = send_pidtune(fd, 0, parameter.as_double());
		          	} else if(parameter.get_name()=="LEFT_KI") {
					    pidtune_result = send_pidtune(fd, 2, parameter.as_double()); 
		          	} else if(parameter.get_name()=="LEFT_KD") {
		          		pidtune_result = send_pidtune(fd, 4, parameter.as_double());
		          	} else if(parameter.get_name()=="RIGHT_KP") {
					    pidtune_result = send_pidtune(fd, 1, parameter.as_double());
		          	} else if(parameter.get_name()=="RIGHT_KI") {
					    pidtune_result = send_pidtune(fd, 3, parameter.as_double());
		          	} else if(parameter.get_name()=="RIGHT_KD") {
					    pidtune_result = send_pidtune(fd, 5, parameter.as_double());
		          	}

		          	if(pidtune_result==0) {
		          		result.successful &= true;
		          	} else {
		          		RCLCPP_ERROR(this->get_logger(), "param_change_callback.send_pidtune returned error: %d", pidtune_result);
		          		result.successful = false;
		          	}

		          }

		        } // for loop end

		        return result;

		      };

		    callback_handler = this->add_on_set_parameters_callback(param_change_callback);

			RCLCPP_INFO(this->get_logger(), "ROSRider node started");

		}

	private:

		    int fd;
		    int rv;
		uint8_t read_state = 0;

		   bool i2c_enabled;
		   bool broadcast_tf2;
		   bool pub_odometry;
		   bool pub_joints;
		   bool pub_diagnostics;
		   bool debug_mode;
		uint8_t ros2rpi_config;

	   unsigned char param_buffer[7] = {0};
	   unsigned char param_result_buffer[4] = {0};
	   unsigned char status_buffer[32] = {0};

	   uint32_t encoder_left;
	   uint32_t encoder_right; 
	   
	     int8_t enc_dir_left;
	     int8_t enc_dir_right;

	   uint16_t packet_age;
	    uint8_t packet_seq;
	    uint8_t prev_packet_seq;
	    uint8_t packet_checksum;

	    std::string odom_frame_id;
	    std::string base_frame_id;
	    
	    uint8_t prev_SYS_STATUS;
		uint8_t prev_PWR_STATUS;
		uint8_t prev_MTR_STATUS;

		rclcpp::Time current_time;
		rclcpp::Time prev_time;
		rclcpp::Time corrected_time;
		rclcpp::Time prev_corrected_time;

		uint32_t i2c_read_status_error_count; 
		uint32_t i2c_read_status_checksum_error_count;

		string sys_status_str = "";
		string power_status_str = "";
		string motor_status_str = "";

		bool sensor_restart_required = false;

		uint32_t param_fail_count = 0;
		uint32_t param_change_count = 0;

		void timer_i2c_callback() {

			current_time = rclcpp::Clock().now();

			uint8_t rs = read_status(fd);
			if(rs != 0) { 
				i2c_read_status_error_count++; 
				if(i2c_read_status_error_count > MAX_I2C_READ_FAIL) { 
					RCLCPP_INFO(this->get_logger(), "read_status_error threshold exceeded");
					exit(0);
				}
			} else { 
				i2c_read_status_error_count = 0; 
			}

			packet_seq = status_buffer[28];								// seq: included in checksum
			packet_age = (status_buffer[29] << 8) | status_buffer[30];	// age: not included in checksum
			packet_checksum = status_buffer[31];						// checksum itself
			
			if(crc8ccitt(status_buffer, 29) == packet_checksum) {

				i2c_read_status_checksum_error_count = 0;

                // if packet sequence has not incremented
                if(packet_seq != (prev_packet_seq + 1)) {

            	    // if not rollover case
                    if(!(packet_seq == 0 and prev_packet_seq == 255)) {

                        if(packet_seq == prev_packet_seq) {

                        	// skip if same packet
                        	RCLCPP_INFO(this->get_logger(), "Repeat SEQ: %d, skipping", packet_seq);

							prev_time = current_time;
							read_state = STATUS_SEQ_REPEAT;

							// reset timer
							// notice: on RPI5 with U24, this errs.
                        	// timer_i2c.reset();
                            return;

                        } else {
                        	// continue with warning
                        	RCLCPP_INFO(this->get_logger(), "Jumped SEQ: %d, Prev: %d", packet_seq, prev_packet_seq);
                            read_state = STATUS_SEQ_JUMP;
                        }
                    }    	
                } else {
                    read_state = STATUS_OK;
                }

				// first calculate odometry
				encoder_left = status_buffer[3] + (status_buffer[2] << 8) + (status_buffer[1] << 16) + (status_buffer[0] << 24);
				encoder_right = status_buffer[7] + (status_buffer[6] << 8) + (status_buffer[5] << 16) + (status_buffer[4] << 24);

				// get encoder directions
				enc_dir_left = (status_buffer[24] & 0x0F) - 1;
	            enc_dir_right = ((status_buffer[24] & 0xF0) >> 4) - 1;

				update_left_wheel(encoder_left);
				update_right_wheel(encoder_right);

				// notice: this is derived from own measurements of this loop
				double delta_seconds = ((double) (current_time.nanoseconds() - prev_time.nanoseconds())) / NS_CONVERSION_CONSTANT;

				update_pose(delta_seconds);
	            quaternion_from_euler(0, 0, pose_theta);      

	            // this is derived over packet age
				rclcpp::Duration time_correction = rclcpp::Duration(0, ((packet_age / 32768.0) * NS_CONVERSION_CONSTANT));

				// this value will be used for stamping
				corrected_time = current_time - time_correction;

                // TODO: document this better. also check for overflow conditions.
				// if timeskip use most recent time
				if(corrected_time < prev_corrected_time) {
					RCLCPP_INFO(this->get_logger(), "Timeskip detected");
					read_state = STATUS_TIMESKIP;
					corrected_time = current_time;
				}

				// record previous correction time
				prev_corrected_time = corrected_time;

				prev_SYS_STATUS = SYS_STATUS;
				prev_PWR_STATUS = PWR_STATUS;
				prev_MTR_STATUS = MTR_STATUS;

	            SYS_STATUS = status_buffer[27];
	            PWR_STATUS = status_buffer[25];
	            MTR_STATUS = status_buffer[26];

	            // notice: we dont need this since sensor is restarted at driver init
	            if(SYS_STATUS != prev_SYS_STATUS) {
	            	print_sys_status(SYS_STATUS);
	            	if(SYS_STATUS & 0x40) { 
					    send_device_reset();
					    rclcpp::sleep_for(2000ms);
	            	}
	            }

	            if(PWR_STATUS != prev_PWR_STATUS) {
	            	print_pwr_status(PWR_STATUS);
	            }

	            if(MTR_STATUS != prev_MTR_STATUS) {
	            	print_mtr_status(MTR_STATUS);
	            }

				if(pub_odometry) {

					auto odom_data = nav_msgs::msg::Odometry();

					odom_data.header.stamp = corrected_time;
		            odom_data.header.frame_id = odom_frame_id;
		            odom_data.child_frame_id = base_frame_id;
		            odom_data.pose.pose.position.x = pose_x;
		            odom_data.pose.pose.position.y = pose_y;

		            // fill in pose
		            odom_data.pose.pose.orientation.x = orientation[0];
		            odom_data.pose.pose.orientation.y = orientation[1];
		            odom_data.pose.pose.orientation.z = orientation[2];
		            odom_data.pose.pose.orientation.w = orientation[3];

		            // twist
		            odom_data.twist.twist.linear.x = pose_xVel;
		            odom_data.twist.twist.angular.z = pose_thetaVel;

		            // covariances
		            odom_data.pose.covariance[0] = 0.01;
		            odom_data.pose.covariance[7] = 0.0;
		            odom_data.pose.covariance[14] = 0.0;
		            odom_data.pose.covariance[21] = 0.0;
		            odom_data.pose.covariance[28] = 0.0;
		            odom_data.pose.covariance[35] = 0.1;

		            odom_data.twist.covariance[0] = 0.01;
		            odom_data.twist.covariance[7] = 0.0;
		            odom_data.twist.covariance[14] = 0.0;
		            odom_data.twist.covariance[21] = 0.0;
		            odom_data.twist.covariance[28] = 0.0;
		            odom_data.twist.covariance[35] = 0.01;	

		            odom_pub->publish(odom_data);		            

				}

				if(broadcast_tf2) {

					auto transform = geometry_msgs::msg::TransformStamped();

					transform.header.stamp = corrected_time;
		            transform.header.frame_id = odom_frame_id;
		            transform.child_frame_id = base_frame_id;
		            transform.transform.translation.x = pose_x;
		            transform.transform.translation.y = pose_y;
		            transform.transform.translation.z = 0.0;
		            transform.transform.rotation.x = orientation[0];
		            transform.transform.rotation.y = orientation[1];
		            transform.transform.rotation.z = orientation[2];
		            transform.transform.rotation.w = orientation[3];

		            tf_broadcaster->sendTransform(transform);

				}

				if(pub_joints) {

					auto joint_states = sensor_msgs::msg::JointState();

					float left_wheel_position = ((encoder_left % PULSE_PER_REV) / (PULSE_PER_REV * 1.0f)) * 2.0f * MATHPI;
		            float right_wheel_position = ((encoder_right % PULSE_PER_REV) / (PULSE_PER_REV * 1.0f)) * 2.0f * MATHPI;
		       
		            joint_states.header.stamp = corrected_time;
		            joint_states.name = { "wheel_left_joint", "wheel_right_joint" };
		            joint_states.position = { left_wheel_position, right_wheel_position };

		            joint_pub->publish(joint_states);
				}

				if(pub_diagnostics) {

					auto diag_message = rosrider_interfaces::msg::Diagnostics();

					diag_message.packet_age = packet_age;

					diag_message.bus_current = (status_buffer[9] + (status_buffer[8] << 8)) / (10.0 * 1000.0);
		            diag_message.bus_voltage = (status_buffer[11] + (status_buffer[10] << 8)) / 1000.0;
		       		diag_message.cs_left = (status_buffer[13] + (status_buffer[12] << 8)) * (6.6 / 4095);
		            diag_message.cs_right = (status_buffer[15] + (status_buffer[14] << 8)) * (6.6 / 4095);
					diag_message.pwm_left = status_buffer[17] + (status_buffer[16] << 8);
					diag_message.pwm_right = status_buffer[19] + (status_buffer[18] << 8);

		            // current rpm raw values, multiply by ROUNDS_PER_MINUTE and apply sign by encoder_dir
		            if(enc_dir_left == -1) {
		                diag_message.rpm_left = -((status_buffer[20] << 8 | status_buffer[21]) * ROUNDS_PER_MINUTE);
		            } else {
		                diag_message.rpm_left = (status_buffer[20] << 8 | status_buffer[21]) * ROUNDS_PER_MINUTE;
		            }
		            if(enc_dir_right == -1) {
		                diag_message.rpm_right = -((status_buffer[22] << 8 | status_buffer[23]) * ROUNDS_PER_MINUTE);
		            } else {
		                diag_message.rpm_right = (status_buffer[22] << 8 | status_buffer[23]) * ROUNDS_PER_MINUTE;
		            }

		            diag_message.target_left = target_left;
		            diag_message.target_right = target_right;

		            diag_message.system_status = SYS_STATUS;
					diag_message.power_status = PWR_STATUS;
		            diag_message.motor_status = MTR_STATUS;

		            diag_pub->publish(diag_message);

				}
 
			} else {
				i2c_read_status_checksum_error_count++;
				read_state = STATUS_CHECKSUM_ERROR;
				RCLCPP_INFO(this->get_logger(), "Checksum Error, Packet Seq: %d", packet_seq);
			}

			if(i2c_read_status_checksum_error_count > MAX_I2C_CHECKSUM_FAIL) {
				RCLCPP_INFO(this->get_logger(), "read_status_checksum_error threshold exceeded");
				exit(0);
			}

			// notice: do not return without these
			prev_packet_seq = packet_seq;
			prev_time = current_time;
            
		}

		uint8_t read_status(int fd) {
			return I2C_RW_Block(fd, 0xA0, I2C_SMBUS_READ, 32, status_buffer);
		}
		
		uint8_t send_parameter_read_result(int fd, uint8_t address) {

			uint8_t rw = I2C_RW_Block(fd, address, I2C_SMBUS_WRITE, 7, param_buffer);
		    if(rw!=0) { return i2c_error_handler(rw); }
		    rclcpp::sleep_for(1ms);

		    uint8_t rd = I2C_RW_Block(fd, 0xB0, I2C_SMBUS_READ, 4, param_result_buffer);
		    rclcpp::sleep_for(1ms);
		    return i2c_error_handler(rd);

		}

		uint8_t send_sysctl(int fd, uint8_t command) {

			unsigned char sysctl_buffer[5] = {0};
			sysctl_buffer[3] = command;

			uint8_t rw = I2C_RW_Block(fd, 0x04, I2C_SMBUS_WRITE, 5, sysctl_buffer);
		    return i2c_error_handler(rw);

		}

		uint8_t i2c_error_handler(uint8_t rw) {
			if(rw == 0) { 
				return 0; 
			} else {
				RCLCPP_INFO(this->get_logger(), "I2C Error: %s", strerror(rw));
				return rw;
			}
		}

		uint8_t send_pidtune(int fd, uint8_t idx, double value) {

			// first byte is idx
			param_buffer[0] = idx;

			// next four bytes are float
			f.f32 = (float) value;
			param_buffer[1] = f.ui8[0];
			param_buffer[2] = f.ui8[1];
			param_buffer[3] = f.ui8[2];
			param_buffer[4] = f.ui8[3];

			// calculate crc
			param_buffer[5] = crc8ccitt(param_buffer, 5);

			uint8_t rw = I2C_RW_Block(fd, 0x08, I2C_SMBUS_WRITE, 7, param_buffer);
			return i2c_error_handler(rw);

		}

	    void cmd_callback(const geometry_msgs::msg::Twist t) const {

	      	// calculate pid targets and apply trim
		    target_left = ((t.linear.x * LINEAR_RPM) - (t.angular.z * ANGULAR_RPM)) * MOTOR_CONSTANT_LEFT;
		    target_right = ((t.linear.x * LINEAR_RPM) + (t.angular.z * ANGULAR_RPM)) * MOTOR_CONSTANT_RIGHT;

		    // if any command is above limit
		    if(max(abs(target_left), abs(target_right)) >  params_float[PARAM_MAX_RPM]) {
		        // calculate factor
		        float factor =  params_float[PARAM_MAX_RPM] / max(abs(target_left), abs(target_right)); 
		        target_left *= factor; 		// multiply target by + factor
		        target_right *= factor;		// targets will retain sign
		    }

	   		unsigned char pid_target_buffer[9] = {0};
			f.f32 = target_left;
			pid_target_buffer[0] = f.ui8[0];
			pid_target_buffer[1] = f.ui8[1];
			pid_target_buffer[2] = f.ui8[2];
			pid_target_buffer[3] = f.ui8[3];
			f.f32 = target_right;
			pid_target_buffer[4] = f.ui8[0];
			pid_target_buffer[5] = f.ui8[1];
			pid_target_buffer[6] = f.ui8[2];
			pid_target_buffer[7] = f.ui8[3];
			
			uint8_t rw = I2C_RW_Block(fd, 0x02, I2C_SMBUS_WRITE, 9, pid_target_buffer);
			if(rw != 0) { 
				RCLCPP_INFO(this->get_logger(), "I2C Error: %s", strerror(rw));
			}

	    }

	    bool process_parameter_result(uint8_t index, const char *names[], const char *value) {

		    if(param_buffer[5] != param_result_buffer[2]) {
		    	param_fail_count++;
		    	RCLCPP_ERROR(this->get_logger(), "%s: %s, Parameter Checksum Error", names[index], value);
		    	return false;
		    } else {
		    	if(param_result_buffer[3] == 0) {	
		    		param_change_count++;
		    		RCLCPP_INFO(this->get_logger(), "%s: %s, Success", names[index], value);
		    		return true;
			    } else if(param_result_buffer[3] == 1){
		    		RCLCPP_INFO(this->get_logger(), "%s: %s, Unmodified", names[index], value);
		    		return true;
			    } else {
			    	param_fail_count++;
			    	RCLCPP_ERROR(this->get_logger(), "%s: %s, Result Error: %d", names[index], value, param_result_buffer[3]);
			    	return false;
			    }
		    }
	    }

	    uint8_t send_hat_command(int fd, uint8_t output) {

	    	int rv_hat = ioctl(fd, I2C_SLAVE, 0x20);
	    	if(rv_hat<0) { return rv_hat; }

            unsigned char hat_command[1] = {0};
            // notice hat_command[0] is 0x0 at this point.
            int rw_hat = I2C_RW_Block(fd, 0x03, I2C_SMBUS_WRITE, 1, hat_command);

            hat_command[0] = output;
            rw_hat = I2C_RW_Block(fd, 0x01, I2C_SMBUS_WRITE, 1, hat_command);

            return rw_hat;
	    }

	    uint8_t send_device_reset(void) {
	    	RCLCPP_INFO(this->get_logger(), "Sending device reset");
	    	return send_sysctl(fd, 0x01);
	    }

		uint32_t send_parameters(int fd) {

			// { index, MSB1, MSB2, MSB3, MSB4, checksum }

			// send uint8 parameters
			for(uint8_t i=0; i < SIZE_PARAMS_UINT8; i++) {

                // do not sent i2c address as standard parameter
			    if(i==PARAM_I2C_ADDRESS) {
			        continue;
			    }

				uint8_t temp_uint8 = this->get_parameter(names_uint8[i]).as_int();

			    param_buffer[0] = i;
			    param_buffer[1] = temp_uint8;
			    param_buffer[5] = crc8ccitt(param_buffer, 5);

			    uint8_t res = send_parameter_read_result(fd, EEPROM_WRITE_UINT8);
			    if(res != 0) { param_fail_count++; continue; }
			    process_parameter_result(i, names_uint8, std::to_string(temp_uint8).c_str());

			}

			// send uint16 parameters
			for(uint8_t i=0; i < SIZE_PARAMS_UINT16; i++) {

				uint16_t temp_uint16 = this->get_parameter(names_uint16[i]).as_int();

			    param_buffer[0] = i;
			    param_buffer[1] = temp_uint16 & 0xFF;
			    param_buffer[2] = (temp_uint16 >> 8) & 0xFF;
			    param_buffer[5] = crc8ccitt(param_buffer, 5);

			    uint8_t res = send_parameter_read_result(fd, EEPROM_WRITE_UINT16);
			    if(res != 0) { param_fail_count++; continue; }
			    process_parameter_result(i, names_uint16, std::to_string(temp_uint16).c_str());

			}

			// send uint32 parameters
			for(uint8_t i=0; i < SIZE_PARAMS_UINT32; i++) {

				uint32_t temp_uint32 = this->get_parameter(names_uint32[i]).as_int();

			    param_buffer[0] = i;
			    u.u32 = temp_uint32;
			    param_buffer[1] = u.ui8[0];
			    param_buffer[2] = u.ui8[1];
			    param_buffer[3] = u.ui8[2];
			    param_buffer[4] = u.ui8[3];
			    param_buffer[5] = crc8ccitt(param_buffer, 5);

			    uint8_t res = send_parameter_read_result(fd, EEPROM_WRITE_UINT32);
			    if(res != 0) { param_fail_count++; continue; }
			    process_parameter_result(i, names_uint32, std::to_string(temp_uint32).c_str());

			}			

			// send int16 parameters
			for(uint8_t i=0; i < SIZE_PARAMS_INT16; i++) {

				int16_t temp_int16 = this->get_parameter(names_int16[i]).as_int();

			    param_buffer[0] = i;
			    param_buffer[1] = temp_int16 & 0xFF;
			    param_buffer[2] = (temp_int16 >> 8) & 0xFF;
			    param_buffer[5] = crc8ccitt(param_buffer, 5);

			    uint8_t res = send_parameter_read_result(fd, EEPROM_WRITE_INT16);
			    if(res != 0) { param_fail_count++; continue; }
			    process_parameter_result(i, names_int16, std::to_string(temp_int16).c_str());

			}		

			// send boolean parameters
			for(uint8_t i=0; i < SIZE_PARAMS_BOOL; i++) {

				bool temp_bool = this->get_parameter(names_bool[i]).as_bool();

			    param_buffer[0] = i;
			    if(temp_bool) { param_buffer[1] = 0x1; } else { param_buffer[1] = 0x0; }
			    param_buffer[5] = crc8ccitt(param_buffer, 5);

			    uint8_t res = send_parameter_read_result(fd, EEPROM_WRITE_BOOL);
			    if(res != 0) { param_fail_count++; continue; }
				process_parameter_result(i, names_bool, std::to_string(temp_bool).c_str());

			}	

			// send float parameters
			for(uint8_t i=0; i < SIZE_PARAMS_FLOAT; i++) {

				float temp_float = this->get_parameter(names_float[i]).as_double();

			    param_buffer[0] = i;
			    f.f32 = temp_float;
			    param_buffer[1] = f.ui8[0];
			    param_buffer[2] = f.ui8[1];
			    param_buffer[3] = f.ui8[2];
			    param_buffer[4] = f.ui8[3];
			    param_buffer[5] = crc8ccitt(param_buffer, 5);

			    uint8_t res = send_parameter_read_result(fd, EEPROM_WRITE_FLOAT);
			    if(res != 0) { param_fail_count++; continue; }
			    process_parameter_result(i, names_float, std::to_string(temp_float).c_str());

			}

			return param_fail_count;
		}	

		void print_sys_status(uint8_t sys_status) {

		    sys_status_str = "SYS_STATUS | ";

		    if(sys_status & 0x80) {
		        sys_status_str.append("EEPROM_INIT_ERROR | ");
		    }
		    if(sys_status & 0x40) {
		        sys_status_str.append("RESTART_REQUIRED | ");
		    }
		    if(sys_status & 0x01) {
		        sys_status_str.append("EEPROM_WRITE_I2C_ERROR | ");
		    }

		    sys_status_str.append(std::to_string(sys_status));
		    RCLCPP_INFO(this->get_logger(), sys_status_str.c_str());

		}

		void print_pwr_status(uint8_t pwr_status) {

		    power_status_str = "POWER_STATUS | ";

		    if(pwr_status & 0x80) {
		        power_status_str.append("CMD_TIMEOUT | ");
		    }

		    if(pwr_status & 0x40) {
		        power_status_str.append("POWER_BAD | ");
		    }

		    if(pwr_status & 0x20) {
		        power_status_str.append("RIGHT_AMP_LIMIT | ");
		    }

		    if(pwr_status & 0x10) {
		        power_status_str.append("LEFT_AMP_LIMIT | ");
		    }                   

		    if(pwr_status & 0x08) {
		        power_status_str.append("MAIN_AMP_LIMIT | ");
		    }

		    if(pwr_status & 0x04) {
		        power_status_str.append("BAT_VOLTS_HIGH | ");
		    }

		    if(pwr_status & 0x02) {
		        power_status_str.append("BAT_VOLTS_LOW | ");
		    }

		    if(pwr_status & 0x01) {
		        power_status_str.append("AUX_CTL_ON | ");
		    }

		    power_status_str.append(std::to_string(pwr_status));

		    RCLCPP_INFO(this->get_logger(), power_status_str.c_str());  
		}

		void print_mtr_status(uint8_t mtr_status) {

		    motor_status_str = "MOTOR_STATUS | ";

		    if(mtr_status & 0x80) {
		        motor_status_str.append("FAULT_RIGHT | ");
		    }

		    if(mtr_status & 0x40) {
		        motor_status_str.append("FAULT_LEFT | ");
		    }

		    if(mtr_status & 0x20) {
		        motor_status_str.append("FWD_RIGHT | ");
		    } else {
		        motor_status_str.append("REV_RIGHT | ");
		    }

		    if(mtr_status & 0x10) {
		        motor_status_str.append("FWD_LEFT | ");
		    } else {
		        motor_status_str.append("REV_LEFT | ");
		    }

		    if(mtr_status & 0x04) {
		        motor_status_str.append("MODE1 | ");
		    }

		    if(mtr_status & 0x08) {
		        motor_status_str.append("MODE2 | ");
		    }

		    uint8_t drive_mode_disp = (mtr_status & 0x01) + (mtr_status & 0x02);
		    motor_status_str.append("DRIVE: ");
		    motor_status_str.append(std::to_string(drive_mode_disp));

			motor_status_str.append(" | ");
		    motor_status_str.append(std::to_string(mtr_status));
		    
		    RCLCPP_INFO(this->get_logger(), motor_status_str.c_str());  

		}		

		rclcpp::TimerBase::SharedPtr timer_i2c;

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
		rclcpp::Publisher<rosrider_interfaces::msg::Diagnostics>::SharedPtr diag_pub;
		rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;

		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
		
		OnSetParametersCallbackHandle::SharedPtr callback_handler;

};

int main(int argc, const char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ROSRider>());
	rclcpp::shutdown();
	return 0;
}