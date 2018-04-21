#ifndef SERVICES_H_
#define SERVICES_H_

#include "double_integrator_dynamics/globals.h"

namespace services {

bool ResetGame(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res);

}  // namespace services

#endif  // SERVICES_H_