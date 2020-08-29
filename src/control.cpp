#include "control.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_control");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    IIwaControl IIC;

    ros::waitForShutdown();

    return 0;   
}