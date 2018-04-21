
#include "double_integrator_dynamics/services.h"

namespace services {

bool ResetGame(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res) {
	// Update reference
	pthread_mutex_lock(&mutexes_.m_did_class);
		globals_.obj_did.DeactivateQuads();
	pthread_mutex_unlock(&mutexes_.m_did_class);

	return true;
}


}  // namespace services