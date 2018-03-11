
#include "double_integrator_dynamics/threads.h"

namespace threads {

void IntegrationThread(const double &rate) {
    ROS_DEBUG("[didynamics]: Integration Thread started!");

    // Rate at which this thread will run
    // double rate = 300.0;  // Rate in Hz
    double dt = 1/rate;
    ros::Rate loop_rate(rate);


    // Run the Mediation Layer loop
    std::set<QuadData>::iterator it1, it2;
    while (ros::ok()) {
        // Get time for when this task started
        const ros::Time t0 = ros::Time::now();

        pthread_mutex_lock(&mutexes_.m_did_class);

            globals_.obj_did.UpdateDIDOutputs(dt);

            globals_.obj_did.PublishPositions();

        pthread_mutex_unlock(&mutexes_.m_did_class);

        // ros::Duration ml_time = ros::Time::now() - t0;
        // ROS_INFO("Mediation Layer execution time: %f", ml_time.toSec());

        loop_rate.sleep();
    }
    ROS_DEBUG("Exiting Mediation Layer Thread...");
}

void HeartbeatThread() {
    ROS_DEBUG("[didynamics]: Heartbeat Thread started!");

    const double rate = 5.0;  // Rate in Hz
    ros::Rate loop_rate(rate);

    // Timeout for heartbeats
    const double timeout = 0.5;

    while (ros::ok()) {
        pthread_mutex_lock(&mutexes_.m_did_class);

        ros::Time time_now = ros::Time::now();
        
        std::set<QuadData>::iterator it;
        for(it = globals_.obj_did.quads_.begin(); 
            it != globals_.obj_did.quads_.end(); ++it) {

            ros::Time last_ref = it->last_reference_stamp;
            if (it->ref_is_active && (time_now - last_ref).toSec() > timeout) {
                it->ref_is_active = false;
                it->reference.Pos = it->vehicle_odom.pose.pose.position;
                it->reference.Vel = helper::SetVector3(0.0, 0.0, 0.0);
                it->reference.Acc = helper::SetVector3(0.0, 0.0, 0.0);
                ROS_WARN("[didynamics] Quad %s references inactive!", it->name.c_str());
            }
        }

        pthread_mutex_unlock(&mutexes_.m_did_class);

        loop_rate.sleep();
    }
}


}  // namespace threads
