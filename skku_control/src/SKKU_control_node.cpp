#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <signal.h>
#include <skku_control/dsr_hw_interface.h>
#include <skku_tools/control_loop.h>
#include <controller_manager/controller_manager.h>
#include <moveit_msgs/CartesianTrajectory.h>

CDRFLEx Drfl_;

using namespace dsr_control;

bool g_nKill_dsr_control = false; 

void SigHandler(int sig);

int main(int argc, char** argv) {
    // CPU binding 

    int core_id = 0;  // cpu core number

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        ROS_ERROR("[dsr_control] Failed to set CPU affinity.");
        return -1;
    }


    // END: CPU binding 

    ros::init(argc, argv, "SKKU_control_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle private_nh("~");
    //ros::NodeHandle traj_nh;
    //dsr_control::DRHWInterface drhw_interface(traj_nh);  // 인스턴스 생성



    signal(SIGINT, SigHandler);
    signal(SIGTERM, SigHandler); 
    signal(SIGQUIT, SigHandler); 


 
    //----- get param ---------------------
    int rate;
    private_nh.param<int>("rate", rate, 100);
    ROS_INFO("rate is %d\n", rate);
    
    ros::Publisher fail_pub = private_nh.advertise<std_msgs::Int32>("/failure_topic", 10);
    //ros::Subscriber traj_sub = traj_nh.subscribe("/cartesian_trajectory", 100, &DRHWInterface::Impedancecontrol_cb,&drhw_interface);
    ros::Rate r(rate);

    ///dsr_control::DRHWInterface arm(nh);
    DRHWInterface* pArm = NULL;
    pArm = new DRHWInterface(private_nh);

    if(!pArm->init() ){
        ROS_ERROR("[dsr_control] Error initializing robot");
        return -1;
    }
    ROS_INFO("RT connection completed");

    ///controller_manager::ControllerManager cm(&arm, nh);
    controller_manager::ControllerManager cm(pArm, private_nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time last_time;
    ros::Time curr_time;
    ros::Duration elapsed;
    last_time = ros::Time::now();

    ROS_INFO("[dsr_control] controller_manager is updating!");
    
    ros::Duration(2.0).sleep();
    while(ros::ok() && (false==g_nKill_dsr_control))
    ///while(g_nKill_dsr_control==false)
    {
        try{
            ///ROS_INFO("[dsr_control] Running...(g_nKill_dsr_control=%d)",g_nKill_dsr_control);
            curr_time = ros::Time::now();
            elapsed = curr_time - last_time;
            if(pArm) pArm->read(elapsed);
            cm.update(ros::Time::now(), elapsed);
            if(pArm) pArm->write(elapsed);
            
    
            std_msgs::Int32 fail_msg;
            fail_msg.data = SKKU::fail;
            fail_pub.publish(fail_msg);
            

            r.sleep();	//(1000/rate)[sec], default: 10ms 
        }
        catch(std::runtime_error& ex)
        {
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            ROS_ERROR("[dsr_control] Exception: [%s]", ex.what());
            break;
        }
    }

    spinner.stop();
    //if(pArm) delete(pArm);

    ROS_INFO("[dsr_control] Good-bye!");

    return 0;
}



// void SigHandler(int sig)
// {
//     // Do some custom action.
//     // For example, publish a stop message to some other nodes.
  
//     // All the default sigint handler does is call shutdown()
//     ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
//     ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
//     ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    
//     g_nKill_dsr_control = true;
//     ///usleep(1000*1000);
//     ros::shutdown();
    
// }
void SigHandler(int sig)
{
    ROS_INFO("[dsr_control] shutdown time! (sig = %d)",sig);
    g_nKill_dsr_control = true;
    Drfl_.set_robot_control(CONTROL_RESET_SAFET_STOP);
    exit(0);  
}
