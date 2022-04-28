    #include "My_Filter.h"
    #include <unistd.h>
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "my_pcl_node");
        usleep(1000);
        std::string car_id;

        if (const char* env_p = std::getenv("CAR_ID"))
	    {
            car_id = env_p;
	    }
        else
        {
            std::cout << "not find ENV CAR_ID" << std::endl;
        }


        My_Filter filter(car_id);

        ros::spin();

        return 0;
    }

