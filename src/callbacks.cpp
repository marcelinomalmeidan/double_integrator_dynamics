
#include "double_integrator_dynamics/callbacks.h"

namespace callbacks {

void PVACallback(const mg_msgs::PVA::ConstPtr& msg,
                 const std::string& quad_name) {
	// Update reference
	pthread_mutex_lock(&mutexes_.m_did_class);
		globals_.obj_did.UpdateQuadReference(quad_name, *msg);
	pthread_mutex_unlock(&mutexes_.m_did_class);
}

}  // namespace callbacks