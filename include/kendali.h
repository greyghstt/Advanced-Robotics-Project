// #include <akuisisi.h>

// float setpoint = 0.0;
// float alt;
// float rawroll, rawpitch, rawyaw;
// float U1, U2, U3, U4;
// float Omega_12, Omega_22, Omega_32, Omega_42;
// float Fz, lb = 0.45/2,  k, b;
// float errorAlt, errorRoll, errorPitch, errorYaw;
// float Kpalt = 10.0, Kialt = 0.0, Kdalt = 3.0;
// float KpRoll = 10.0, KpPitch = 10.0, KpYaw = 10.0;
// float KiRoll = 0.0, KiPitch = 0.0, KiYaw = 0.0;
// float KdRoll = 3.0, KdPitch = 3.0, KdYaw = 3.0;
// float integralRoll = 0.0, integralPitch = 0.0, integralYaw = 0.0;
// float previousErrorRoll = 0.0, previousErrorPitch = 0.0, previousErrorYaw = 0.0;// Assuming a time step of 0.01 seconds
// const double A_inverse [4][4] = {
//     {b, -lb, 0, k},
//     {b, 0, lb, -k},
//     {b, lb, 0, k},
//     {b, 0, -lb, -k}
// };

// void kendali (){
//     rawroll = Roll;
//     rawpitch = Pitch;
//     rawyaw = Yaw;
    
//     errorAlt = setpoint - alt;
//     errorRoll = setpoint - rawroll;
//     errorPitch = setpoint - rawpitch;
//     errorYaw = setpoint - rawyaw;
    
//     integralRoll += errorRoll * dt;
//     integralPitch += errorPitch * dt;
//     integralYaw += errorYaw * dt;
    
//     float derivativeRoll = (errorRoll - previousErrorRoll) / dt;
//     float derivativePitch = (errorPitch - previousErrorPitch) / dt;
//     float derivativeYaw = (errorYaw - previousErrorYaw) / dt;
    
//     U1 = Kpalt * errorAlt + Kialt * errorAlt * dt + Kdalt * errorAlt / dt;
//     U2 = KpRoll * errorRoll + KiRoll * integralRoll + KdRoll * derivativeRoll;
//     U3 = KpPitch * errorPitch + KiPitch * integralPitch + KdPitch * derivativePitch;
//     U4 = KpYaw * errorYaw + KiYaw * integralYaw + KdYaw * derivativeYaw;
    
//     Omega_12 = (Fz/(4*b)) + (U4/(4*k)) - (U2/(2*lb));
//     Omega_22 = (Fz/(4*b)) + (U4/(4*k)) + (U3/(2*lb));
//     Omega_32 = (Fz/(4*b)) + (U4/(4*k)) + (U2/(2*lb));
//     Omega_42 = (Fz/(4*b)) - (U4/(4*k)) - (U3/(2*lb));
    
//     previousErrorRoll = errorRoll;
//     previousErrorPitch = errorPitch;
//     previousErrorYaw = errorYaw;
// }