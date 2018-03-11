#ifndef THREADS_H_
#define THREADS_H_

#include "double_integrator_dynamics/globals.h"

namespace threads {

// Thread that runs the mediation layer algorithm
void IntegrationThread(const double &rate);

// Thread for verifying heartbeat of quad references and position
void HeartbeatThread();

}  // namespace threads

#endif  // THREADS_H_