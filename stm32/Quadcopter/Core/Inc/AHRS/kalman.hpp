/*
 * kalman.hpp
 *
 *  Created on: Jun 24, 2024
 *      Author: Louis
 */

// https://ahrs.readthedocs.io/en/latest/filters/ekf.html

#pragma once

// Project
#include "AHRS/ahrs.hpp"
#include "Utils/Eigen/Dense"

//https://codingcorner.org/extended-kalman-filter-in-cpp-with-eigen3/


// the kalman filter choose the kalman gain K, such that P is minimized.

/*
 * Extended Kalman filter
 * The state is a vector 10 ( quaternion (4), gyro (3), gyro's offset (3) )
 */


/*
 * Extended Kalman filter for attitude estimation.
 * Fuse accelerometer, gyroscope and magnetometer data.
 * Work with quaternions.
 */
class ExtendedKalmanFilter : public IFilter
{
private:
	// State vector (quaternion (4) + gyro offset (3))
	Eigen::Vector<double, 7> m_stateEstimate_X;
	Eigen::Matrix<double, 7, 7> m_errorCovariance_P; // Covariance matrix
	Eigen::Matrix<double, 7, 7> m_noiseCovarience_Q; // Process noise covariance
	Eigen::Matrix<double, 4, 4> m_stateTransition_A;

	Eigen::Matrix<double, 6, 6> m_measurementNoiseCovarience_R; // Measurement noise covariance



public:
    ExtendedKalmanFilter()
    {
    	// Initial state - quaternion (w, x, y, z)
    	// Sets to identity quaternion
    	m_stateEstimate_X << 1, 0, 0, 0, 0, 0, 0;

    	m_errorCovariance_P.setIdentity(); // Initial state covariance
    	m_noiseCovarience_Q.setIdentity(); // Process noise covariance, scaled as needed
    	m_noiseCovarience_Q *= 0.1;

    	m_measurementNoiseCovarience_R.setIdentity(); // Measurement noise covariance matrix for 6 measurements
    	m_measurementNoiseCovarience_R *= 0.5;

		// Set initial values for H. Update based on how measurements relate to the states
		//updateH();
    }

    Quaternion compute(
    		const Vector<double, 3>& acc,
			const Vector<double, 3>& gyro,
			const Vector<double, 3>& magneto,
			const double& dt
			) override
    {
    	Eigen::Vector3d _gyro(gyro.m_vect[0], gyro.m_vect[1], gyro.m_vect[2]);
        // Prediction
        predict(_gyro * DEGREE_TO_RAD, dt);

        // Update with accelerometer and magnetometer
        Eigen::Vector<double, 6> measurementVector(
        		acc.m_vect[0],
				acc.m_vect[1],
				acc.m_vect[2],
				magneto.m_vect[0],
				magneto.m_vect[1],
				magneto.m_vect[2]
				);
        update(measurementVector);

        // Return the estimated attitude
        return getEstimateAttitude();
    }


    /*
     * First step: the quaternion is predicted by integrating the angular rate
     * Process noise covarience
     */
    void predict(const Eigen::Vector3d& gyro, const double& dt)
    {
    	// Quaternion integration
    	m_stateTransition_A = computeStateTransitionMatrix(gyro, dt);
    	Eigen::Vector4d q = m_stateTransition_A * m_stateEstimate_X.head<4>(); // q = A * [x(0), x(1), x(2), x(3)]

    	// Normalize quaternion
    	q.normalize();

    	// Update quaternion part of the state vector
    	m_stateEstimate_X(0) = q(0);
    	m_stateEstimate_X(1) = q(1);
    	m_stateEstimate_X(2) = q(2);
    	m_stateEstimate_X(3) = q(3);


		// Jacobian of the state transition matrix
    	Eigen::Matrix<double, 7, 7> F;
		F = computeJacobianStateTransitionModel();

		// Predict error covariance
		m_errorCovariance_P = F * m_errorCovariance_P * F.transpose() + m_noiseCovarience_Q;
    }

    /*
     * Correction step
     * Use accelerometer and magnetometer to correct the gyro estimation
     */
    void update(const Eigen::Vector<double, 6>& z)
    {
    	// Compute H based on the current state vector
    	// Jacobian of the measurement function matrix
    	Eigen::Matrix<double, 6, 7> H = computeJacobianMeasurementModel_H(z);

    	Eigen::Matrix<double, 6, 6> S = H * m_errorCovariance_P * H.transpose() + m_measurementNoiseCovarience_R;
    	// Kalman gain
    	Eigen::Matrix<double, 7, 6> K;
    	K = m_errorCovariance_P * H.transpose() * S.inverse();

    	// Compute expected measurement from accelerometer and magnetometer
    	Eigen::Vector<double, 6> h = computeMeasurementEstimate_h();

    	// Innovation (residual)
    	Eigen::Vector<double, 6> y = z - h;

    	// Update state estimate
    	m_stateEstimate_X = m_stateEstimate_X + K * y;

    	// Normalize output quaternion
    	Eigen::Vector4d q_updated = m_stateEstimate_X.head<4>();
    	q_updated.normalize();
    	m_stateEstimate_X.head<4>() = q_updated;

    	// Update error covariance matrix
    	Eigen::Matrix<double, 7, 7> I;
    	I.setIdentity();
    	m_errorCovariance_P = (I - K * H) * m_errorCovariance_P;
    }

    Quaternion getEstimateAttitude() const
    {
    	// TODO: m_stateEstimate_X.head<4>()
        return Quaternion(m_stateEstimate_X(0), m_stateEstimate_X(1), m_stateEstimate_X(2), m_stateEstimate_X(3));
    }



    /*
     * Compute the state transition matrix
     */
    virtual Eigen::Matrix<double, 4, 4> computeStateTransitionMatrix(const Eigen::Vector3d& gyro, const double& dt)
    {
    	// (Wx, Wy, Wz): angular rate

		//     1     -(dt/2)Wx  -(dt/2)Wy  -(dt/2)Wz
		// (dt/2)Wx       1      (dt/2)Wz  -(dt/2)Wy
		// (dt/2)Wy  -(dt/2)Wz       1      (dt/2)Wx
		// (dt/2)Wz   (dt/2)Wy  -(dt/2)Wx       1


		// Remove the estimated gyro offset (bias) from raw gyro
		double wx = gyro(0) - m_stateEstimate_X(4);
		double wy = gyro(1) - m_stateEstimate_X(5);
		double wz = gyro(2) - m_stateEstimate_X(6);

		// Construct state transition matrix
		Eigen::Matrix4d Ac;
		Ac << 0.0, -wx, -wy, -wz,
				wx, 0.0, wz, -wy,
				wy, -wz, 0.0, wx,
				wz, wy, -wx, 0.0;
		Ac = 0.5 * Ac;

		return Eigen::Matrix4d::Identity() + Ac * dt;
    }


    /*
     * Compute the Jaconian matrix of the state transition model.
     * Matrix of partial derivative.
     * It's purpose is to linearize non-linear systems
     */
    virtual Eigen::Matrix<double, 7, 7> computeJacobianStateTransitionModel()
    {
    	Eigen::Matrix<double, 7, 7> F;

    	F.setZero();
    	F.topLeftCorner<4, 4>() = m_stateTransition_A;  // State transition matrix for quaternion
    	F.bottomRightCorner<3, 3>().setIdentity();  // Identity for gyro offstes

    	return F;
    }


	/*
	 * Compute the measurement model h.
	 * h is not linear so we compute it's jacobian H.
	 * Matrix that map the measurement vector.
	 * Rows = Size of measurement vector
     * Columns = Size of state vector
	 */
    virtual Eigen::Matrix<double, 6, 7> computeJacobianMeasurementModel_H(const Eigen::Vector<double, 6>& z)
	{
		// ENU reference is used here.
		// ENU defines the (x, y, z) axis colinear to the geographical
		// East, North, and Up directions, respectively.

		const Eigen::Vector3d g(0.0, 0.0, 1.0); // Gravity reference vector

		// TODO: measure value
		const Eigen::Vector3d r(0.0, 1.0, 0.0); // Earth magnetic field reference vector

		// Normalize accelerometer and magnetometer's reading
		Eigen::Vector3d a(z(0), z(1), z(2));
		a.normalize();
		Eigen::Vector3d m(z(0), z(1), z(2));
		m.normalize();

		const double g_x = g(0), g_y = g(1), g_z = g(2);
		const double r_x = r(0), r_y = r(1), r_z = r(2);
		const double q_w = m_stateEstimate_X(0);
		const double q_x = m_stateEstimate_X(1);
		const double q_y = m_stateEstimate_X(2);
		const double q_z = m_stateEstimate_X(3);

		// Create the jacobian of the measurement matrix
		// The topLeft<6, 4> map the quaternion, and the rest that is 0 map the gyro offset
		Eigen::Matrix<double, 6, 7> H;
		H.setZero();

		H << g_x*q_w + g_y*q_z - g_z*q_y, g_x*q_x - g_y*q_y - g_z*q_z, g_x*q_y + g_y*q_x + g_z*q_w, g_x*q_z - g_y*q_w + g_z*q_x, 0.0, 0.0, 0.0,
		     -g_x*q_z + g_y*q_w + g_z*q_x, g_x*q_y + g_y*q_x - g_z*q_w, -g_x*q_w + g_y*q_z - g_z*q_y, g_x*q_x + g_y*q_y + g_z*q_z, 0.0, 0.0, 0.0,
		     g_x*q_y - g_y*q_x + g_z*q_w, g_x*q_z + g_y*q_w - g_z*q_x, g_x*q_w - g_y*q_z - g_z*q_y, g_x*q_x + g_y*q_y + g_z*q_z, 0.0, 0.0, 0.0,
		     r_x*q_w + r_y*q_z - r_z*q_y, r_x*q_x - r_y*q_y - r_z*q_z, r_x*q_y + r_y*q_x + r_z*q_w, r_x*q_z - r_y*q_w + r_z*q_x, 0.0, 0.0, 0.0,
		     -r_x*q_z + r_y*q_w + r_z*q_x, r_x*q_y + r_y*q_x - r_z*q_w, -r_x*q_w + r_y*q_z - r_z*q_y, r_x*q_x + r_y*q_y + r_z*q_z, 0.0, 0.0, 0.0,
		     r_x*q_y - r_y*q_x + r_z*q_w, r_x*q_z + r_y*q_w - r_z*q_x, r_x*q_w - r_y*q_z - r_z*q_y, r_x*q_x + r_y*q_y + r_z*q_z, 0.0, 0.0, 0.0;


		H = 2.0 * H;

		return H;
	}


    /*
     * Compute the expected accelerometer and magnetometer readings from current quaternion
     */
    virtual Eigen::Vector<double, 6> computeMeasurementEstimate_h()
    {
    	// Current quaternion
		Eigen::Quaterniond q(m_stateEstimate_X(0), m_stateEstimate_X(1), m_stateEstimate_X(2), m_stateEstimate_X(3));

		// Gravity vector in ENU frame
		Eigen::Vector3d g(0.0, 0.0, 1.0);

		// Earth magnetic field vector in ENU frame TODO
		Eigen::Vector3d r(0.0, 1.0, 0.0);

		// Transform the gravity vector by the quaternion
		Eigen::Vector3d accExpected = q * g;

		// Transform the magnetic field vector by the quaternion
		Eigen::Vector3d magExpected = q * r;

		Eigen::Vector<double, 6> h;
		h << accExpected, magExpected;

		return h;
    }
};
