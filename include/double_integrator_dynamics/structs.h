#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <thread>

struct globalVariables {
    // Mutex protected variables
    DoubleIntegratorDynamics obj_did;
};

class mutexStruct {
 public:
    pthread_mutex_t m_did_class;

    // Methods
    mutexStruct() {
        pthread_mutex_init(&m_did_class, NULL);
    }
    void destroy() {
        pthread_mutex_destroy(&m_did_class);
    }
};

#endif  // STRUCTS_H_