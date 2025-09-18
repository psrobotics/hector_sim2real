#ifndef EKF_H
#define EKF_H

#include <cmath>

/**
 * @class EKF
 * @brief A minimal Extended Kalman Filter for IMU roll and pitch estimation.
 * * This filter fuses gyroscope data (for high-frequency changes) and
 * accelerometer data (for a low-frequency, gravity-based reference) to provide
 * a stable estimate of the roll (phi) and pitch (theta) angles.
 * * State vector x: [roll (phi), pitch (theta)]
 * Process model: Euler angle kinematics from gyro rates (p, q, r)
 * Measurement model: Roll and pitch calculated from accelerometer (ax, ay, az)
 */
class EKF {
public:
    /**
     * @brief Constructor that initializes the filter's state and noise matrices.
     */
    EKF() {
        // State vector [roll, pitch] initialized to zero
        x[0] = 0.0f;
        x[1] = 0.0f;

        // State covariance matrix P: Represents the uncertainty of the state estimate.
        // Initialized to a high value, indicating high initial uncertainty.
        P[0][0] = 1.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 1.0f;

        // Process noise covariance Q: Uncertainty in the process model (gyro integration).
        // Tunes how much you trust the gyro. Smaller values = more trust.
        float Q_angle = 4e-10f;
        Q[0][0] = Q_angle; Q[0][1] = 0.0f;
        Q[1][0] = 0.0f; Q[1][1] = Q_angle;

        // Measurement noise covariance R: Uncertainty in the measurement (accelerometer).
        // Tunes how much you trust the accelerometer. Smaller values = more trust.
        float R_angle = 5e-7f;
        R[0][0] = R_angle; R[0][1] = 0.0f;
        R[1][0] = 0.0f; R[1][1] = R_angle;
    }

    /**
     * @brief Initializes the filter's state with known starting angles.
     * @param initial_roll The initial roll angle in radians.
     * @param initial_pitch The initial pitch angle in radians.
     */
    void init(float initial_roll, float initial_pitch) {
        x[0] = initial_roll;
        x[1] = initial_pitch;
    }

    /**
     * @brief Performs one full prediction and update step of the EKF.
     * @param p Gyro x-axis reading (roll rate) in rad/s.
     * @param q Gyro y-axis reading (pitch rate) in rad/s.
     * @param r Gyro z-axis reading (yaw rate) in rad/s.
     * @param ax Accel x-axis reading in m/s^2.
     * @param ay Accel y-axis reading in m/s^2.
     * @param az Accel z-axis reading in m/s^2.
     * @param dt Time step in seconds.
     */
    void update(float p, float q, float r, float ax, float ay, float az, float dt) {
        // --- 1. PREDICTION STEP ---
        float phi = x[0];
        float theta = x[1];

        // State prediction using gyro data (kinematic model)
        float phi_dot = p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r;
        float theta_dot = cos(phi) * q - sin(phi) * r;
        x[0] += phi_dot * dt;
        x[1] += theta_dot * dt;

        // Jacobian of the state transition model (A)
        float A[2][2];
        A[0][0] = 1.0f + dt * (cos(phi)*tan(theta)*q - sin(phi)*tan(theta)*r);
        A[0][1] = dt * (sin(phi)/pow(cos(theta), 2)*q + cos(phi)/pow(cos(theta), 2)*r);
        A[1][0] = dt * (-sin(phi)*q - cos(phi)*r);
        A[1][1] = 1.0f;

        // Covariance prediction: P = A*P*A^T + Q
        float P_pred[2][2];
        float temp_P[2][2];
        // temp_P = A * P
        temp_P[0][0] = A[0][0]*P[0][0] + A[0][1]*P[1][0];
        temp_P[0][1] = A[0][0]*P[0][1] + A[0][1]*P[1][1];
        temp_P[1][0] = A[1][0]*P[0][0] + A[1][1]*P[1][0];
        temp_P[1][1] = A[1][0]*P[0][1] + A[1][1]*P[1][1];
        // P = temp_P * A^T + Q   (NOTE the transpose)
        P[0][0] = temp_P[0][0]*A[0][0] + temp_P[0][1]*A[0][1] + Q[0][0];
        P[0][1] = temp_P[0][0]*A[1][0] + temp_P[0][1]*A[1][1] + Q[0][1];
        P[1][0] = temp_P[1][0]*A[0][0] + temp_P[1][1]*A[0][1] + Q[1][0];
        P[1][1] = temp_P[1][0]*A[1][0] + temp_P[1][1]*A[1][1] + Q[1][1];

        // --- 2. UPDATE STEP ---
        // Measurement from accelerometer
        float phi_acc = atan2(ay, az);
        float theta_acc = atan2(-ax, sqrt(ay*ay + az*az));
        float z[2] = {phi_acc, theta_acc};
        
        // Measurement model is identity: h(x) = x, so Jacobian H = I
        // Measurement residual: y = z - h(x)
        float y[2] = {z[0] - x[0], z[1] - x[1]};

        // Measurement residual covariance: S = H*P*H^T + R = P + R
        float S[2][2];
        S[0][0] = P[0][0] + R[0][0]; S[0][1] = P[0][1] + R[0][1];
        S[1][0] = P[1][0] + R[1][0]; S[1][1] = P[1][1] + R[1][1];

        // Inverse of S
        float S_inv[2][2];
        float det_S = S[0][0]*S[1][1] - S[0][1]*S[1][0];
        if (fabs(det_S) < 1e-9) det_S = 1e-9; // Avoid division by zero
        S_inv[0][0] = S[1][1] / det_S;  S_inv[0][1] = -S[0][1] / det_S;
        S_inv[1][0] = -S[1][0] / det_S; S_inv[1][1] = S[0][0] / det_S;

        // Kalman Gain: K = P*H^T*S^-1 = P*S^-1
        float K[2][2];
        K[0][0] = P[0][0]*S_inv[0][0] + P[0][1]*S_inv[1][0]; K[0][1] = P[0][0]*S_inv[0][1] + P[0][1]*S_inv[1][1];
        K[1][0] = P[1][0]*S_inv[0][0] + P[1][1]*S_inv[1][0]; K[1][1] = P[1][0]*S_inv[0][1] + P[1][1]*S_inv[1][1];
        
        // Update state estimate: x = x + K*y
        x[0] += K[0][0]*y[0] + K[0][1]*y[1];
        x[1] += K[1][0]*y[0] + K[1][1]*y[1];

        // Update error covariance: P = (I - K*H)*P = (I - K)*P
        float I_K[2][2];
        I_K[0][0] = 1.0f - K[0][0]; I_K[0][1] = -K[0][1];
        I_K[1][0] = -K[1][0];       I_K[1][1] = 1.0f - K[1][1];
        
        float P_new[2][2];
        P_new[0][0] = I_K[0][0]*P[0][0] + I_K[0][1]*P[1][0]; P_new[0][1] = I_K[0][0]*P[0][1] + I_K[0][1]*P[1][1];
        P_new[1][0] = I_K[1][0]*P[0][0] + I_K[1][1]*P[1][0]; P_new[1][1] = I_K[1][0]*P[0][1] + I_K[1][1]*P[1][1];
        P[0][0] = P_new[0][0];  P[0][1] = P_new[0][1];
        P[1][0] = P_new[1][0];  P[1][1] = P_new[1][1];
    }
    
    float getRoll() const { return x[0]; }
    float getPitch() const { return x[1]; }

private:
    float x[2];      // State vector [phi, theta]
    float P[2][2];   // State covariance
    float Q[2][2];   // Process noise covariance
    float R[2][2];   // Measurement noise covariance
};

#endif // EKF_H