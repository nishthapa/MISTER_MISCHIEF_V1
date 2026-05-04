#include "behaviours/Mode_MaintainDistance.h"

Mode_MaintainDistance::Mode_MaintainDistance(HCSR04_Sonar* s, XY160D_MotorDriver* m, PIDController* p) {
    sonar = s; motors = m; pid = p;
}

void Mode_MaintainDistance::update(const RobotMood& currentMood) {
    float currentDistance = sonar->getDistanceCM();
    
    // Target is exactly 15.0 cm
    // If distance is 10cm (too close), error is positive -> robot backs up.
    // If distance is 20cm (too far), error is negative -> robot moves forward.
    float correction = pid->compute(15.0f, currentDistance, 0.01f); 
    
    float finalSpeed = correction * currentMood.speedMultiplier;
    motors->drive(-finalSpeed, -finalSpeed); 
}