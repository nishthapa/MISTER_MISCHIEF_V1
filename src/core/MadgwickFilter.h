#pragma once

class MadgwickFilter {
private:
    float beta;
    float q0, q1, q2, q3; 

    // Hidden math functions
    void compute6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void compute9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

public:
    MadgwickFilter(float beta);

    // The Unified Interface: Give it everything. It will auto-route based on the hasCompass flag!
    void compute(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, bool hasCompass);

    void updateBeta(float newBeta) { beta = newBeta; }

    float getRoll();
    float getPitch();
    float getYaw();
};