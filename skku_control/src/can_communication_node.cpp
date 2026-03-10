#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define SENSOR_ID_1 0x01  // 첫 번째 센서 ID
#define SENSOR_ID_2 0x02  // 두 번째 센서 ID
#define INDEX_ID 0x102
#define SENSOR_ID 0x01

void transmit_mode(int s) {
    struct can_frame frame;
    frame.can_id = INDEX_ID; // 모드 변경 명령을 보낼 ID
    frame.can_dlc = 8;
    frame.data[0] = SENSOR_ID; // 모드 변경 명령어
    frame.data[1] = 0x03; // 모드 변경 데이터
    frame.data[2] = 0x01;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Error changing mode");
    } else {
        ROS_INFO("Transmit mode command sent successfully.");
    }
}

bool initialize(int s) {
    struct can_frame frame;
    frame.can_id = INDEX_ID; // 모드 변경 명령을 보낼 ID
    frame.can_dlc = 8;
    frame.data[0] = SENSOR_ID; // 모드 변경 명령어
    frame.data[1] = 0x02; // 모드 변경 데이터
    frame.data[2] = 0x01;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Error changing mode");
        return false; // 초기화 실패
    } else {
        ROS_INFO("Initialize command sent successfully.");
        return true; // 초기화 성공
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "can_communication_node"); 
    ros::NodeHandle nh; 
    ros::Publisher sensor_pub = nh.advertise<std_msgs::Float32MultiArray>("/sensor_data", 10); 

    int s; 
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return 1;
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr); 

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return 1;
    }

    if (initialize(s)) { // initialize가 성공했을 때만 transmit_mode 실행
        usleep(1000000);
        transmit_mode(s);
    } else {
        ROS_ERROR("Failed to initialize. Exiting.");
        close(s);
        return 1;
    }

    std_msgs::Float32MultiArray sensor_msg; 
    sensor_msg.data.resize(6); 

    while (ros::ok()) {
        ssize_t nbytes = read(s, &frame, sizeof(struct can_frame)); 
        if (nbytes < 0) {
            perror("Can read");
            break;
        } else if (nbytes == sizeof(struct can_frame)) {
            if (frame.can_id == SENSOR_ID_1) {
                sensor_msg.data[0] = (static_cast<int>(frame.data[0]) * 256 + static_cast<int>(frame.data[1])) / 100.0 - 300.0;
                sensor_msg.data[1] = (static_cast<int>(frame.data[2]) * 256 + static_cast<int>(frame.data[3])) / 100.0 - 300.0;
                sensor_msg.data[2] = (static_cast<int>(frame.data[4]) * 256 + static_cast<int>(frame.data[5])) / 100.0 - 300.0;
            } 
            else if (frame.can_id == SENSOR_ID_2) {
                sensor_msg.data[3] = (static_cast<int>(frame.data[0]) * 256 + static_cast<int>(frame.data[1])) / 500.0 - 50.0;
                sensor_msg.data[4] = (static_cast<int>(frame.data[2]) * 256 + static_cast<int>(frame.data[3])) / 500.0 - 50.0;
                sensor_msg.data[5] = (static_cast<int>(frame.data[4]) * 256 + static_cast<int>(frame.data[5])) / 500.0 - 50.0;
            }

            sensor_pub.publish(sensor_msg); 
        }
        ros::spinOnce();
    }

    close(s); 
    return 0;
}
