
#include "vision_node.h"
#include "armor/constraint_set.h"
#include "pnp_solver.h"
#include "gimbal/gimbal_control.h"

#include "mercure/mercure_driver.h"

#define BUFFER_SIZE 3

volatile unsigned int prdIdx = 0;
volatile unsigned int csmIdx = 0;

cv::Mat capturedata;   // Buffer of capture

void VisionNode::ImageProducer() 
{
	camera::MercureDriver driver;

	while (true) {
		while (prdIdx - csmIdx >= BUFFER_SIZE);
		driver.ReadCamera(capturedata);
		//cv::imshow("img", capturedata);
		//cv::waitKey(1);
		++prdIdx;
	}
}

void VisionNode::ImageConsumer() 
{
	// Init
	std::vector<ArmorInfo> armors;
	ConstraintSet armor_detector; // task1

	std::vector<cv::Point3f> targets_3d;

	GimbalContrl gimbal_contrl;

	PnpSolver angle_solver_armor_small(124, 54);
	PnpSolver angle_solver_armor_big(216, 54);

	cmd_mode_ = 1;

	// Loop
    while (true) {
		auto speed_test_start_begin_time = std::chrono::steady_clock::now();
		// get camera image ready
        while (prdIdx - csmIdx == 0);

			capturedata.copyTo(armor_detector.src_img_);
			//armor_detector.src_img_ = capturedata;
			++csmIdx;
			
			armors.clear();
			targets_3d.clear();

			// vision detect
			armor_detector.Detect(ros_info_to_car_.vision_data_status, armors);

			for (int idx = 0; idx != armors.size(); ++idx)
			{
				cv::Point3f target_3d;
				bool shootable = angle_solver_armor_small.GetXYZ(armors[idx].rect, target_3d);
				
				if(shootable)
				{
					targets_3d.push_back(target_3d);
				}
			}

			// decision
			if (targets_3d.size())
			{
				ros_info_to_car_.vision_data_x = targets_3d[0].x;
				ros_info_to_car_.vision_data_y = targets_3d[0].y;
				ros_info_to_car_.vision_data_z = targets_3d[0].z;

				gimbal_contrl.SolveContrlAgnle(targets_3d[0], 
								               ros_info_to_car_.cmd_gimbal_yaw, 
								               ros_info_to_car_.cmd_gimbal_pitch);

				//ROS_WARN("yaw = %f, pitch = %f",  ros_info_to_car_.cmd_gimbal_yaw, ros_info_to_car_.cmd_gimbal_pitch);
			}

			// ros publish
			ros_info_to_car_.task_id            = 1;
			ros_info_to_car_.vision_data_status = 1;
			ros_info_to_car_.cmd_mode           = 1;
			

			ros_pub_vison_data_.publish(ros_info_to_car_);
			//ROS_INFO("I push the vision data.");
			
			// show
			auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
			if(!show_window_)
			{
				char str[100];
				sprintf(str, "x = %.2f, y = %.2f, z = %.2f, time = %.2f ms", 
				ros_info_to_car_.vision_data_x,
				ros_info_to_car_.vision_data_y,
				ros_info_to_car_.vision_data_z,
				cost);
				putText(armor_detector.src_img_, str, Point(5, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0,205,0), 1);

				//cv::imshow("relust_img_", armor_detector.src_img_);   // 用于颜色debug
				//cv::imshow("gray_img_", armor_detector.gray_img_);  // 用于灰度debug
				//cv::waitKey(1);
			}
			else ROS_INFO("time cost = %.2f ms........", cost);		
	}
}

void VisionNode::CmdCallBack(const roborts_msgs::InfoFromCar::ConstPtr & msg)
{
	//ROS_WARN("I listening CMD.");
	robot_id_ = msg->robot_id;     // 机器人id
	cmd_mode_ = msg->cmd_mode;     // 命令模式
}

int main(int argc, char *argv[])
{
	capturedata.create(720, 1280, CV_8UC3);

    ros::init(argc, argv, "roborts_vision");
    
    VisionNode vision_node;

    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    ros::waitForShutdown();
 	
	return 0;
}
