#include <test_bed_core/gripper_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_gripper");
    ros::NodeHandle nh;

    GripperInterface gi(nh);
    gi.homeGripper();
    //float f = std::stof(std::string(argv[1]));
    //gi.setForceLimit(f);
    gi.closeGripper(40, 20);
    gi.homeGripper();

    return 0;
}
