#include "twist_chassis.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_twist_node");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(60);

	TwistChassis controller(node_handle, "twist_chassis");
	while (ros::ok())
	{
		for (auto i : range(0, 360).step(6))
		{
			if (true == controller.twist_enable)
			{
				controller.send_chassis_cmd(i);
				controller.send_gimbal_cmd();
			}
			loop_rate.sleep();
			ros::spinOnce();
		}
	}

	return 0;
}
