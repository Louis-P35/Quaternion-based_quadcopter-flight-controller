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
#include "Utils/utilsAlgebra.hpp"

// External library
#include "Utils/Eigen/Dense"


#define INITIAL_GYRO_BIAS 0.0
#define NOISE_COVARIANCE_SCALE 0.1
#define MEASUREMENT_NOISE_COVARIANCE_SCALE 0.5

// TODO:
// why computeJacobianStateTransitionModel() == m_stateTransition_A ?


// Theory:
// https://codingcorner.org/extended-kalman-filter-in-cpp-with-eigen3/


// the kalman filter choose the kalman gain K, such that P is minimized.

/*
 * Extended Kalman filter
 * The state is a vector 7 ( quaternion (4), gyro's offset (3) )
 */

// 1: calcul of the Kalman Gain
// Kalman_Gain = error_estimate / (error_estimate + error_measurement)
// K close to 1 => error_estimate is large comparing to the error_measurement
// K close to 0 => error_measurement is large comparing to the error_estimate

// 2: calcul of the estimate
// estimate(t) = estimate(t-1) + K*(measured_valu - estimate(t-1))

// 3: Recalculate the error in the estimate compared to the previous
// Error_estimate(t) = (1 - K)*error_estimate(t-1)
// k close to 1 -> error_measurement is small -> error_estimate will be small to converge fast
// k close to 0 -> error_measurement is large -> error_estimate will not change quickly



/*
 * Extended Kalman filter for attitude estimation.
 * Fuse accelerometer, gyroscope and magnetometer data.
 * Work with quaternions.
 */

//template<typename T, size_t StateVectSize, size_t MeasureVectSize>
#define StateVectSize 4  // XXX
#define MeasureVectSize 6
class ExtendedKalmanFilter : public IFilter
{
private:
	// State vector (quaternion (4) + gyro offset (3))
	// State estimate vector, the value that the kalman filter estimate
	Eigen::Vector<double, StateVectSize> m_stateEstimate_X;

	// Error covariance matrix, represent the uncertainty in the state estimate
	Eigen::Matrix<double, StateVectSize, StateVectSize> m_errorCovariance_P;

	// Noise covariance matrix, represent the uncertainty in the process model
	Eigen::Matrix<double, StateVectSize, StateVectSize> m_noiseCovarience_Q;


	Eigen::Matrix<double, StateVectSize, StateVectSize> m_stateTransition_A;

	Eigen::Matrix<double, MeasureVectSize, MeasureVectSize> m_measurementNoiseCovarience_R; // Measurement noise covariance



public:
    ExtendedKalmanFilter()
    {
    	// Initial state - quaternion (w, x, y, z)
    	// Sets to identity quaternion
    	m_stateEstimate_X << 1, 0, 0, 0; //, 0, 0, 0;  // XXX

    	// Initial state covariance
    	m_errorCovariance_P.setIdentity();

    	// Process noise covariance, scaled as needed
    	m_noiseCovarience_Q.setIdentity();
    	m_noiseCovarience_Q *= NOISE_COVARIANCE_SCALE;

    	// Measurement noise covariance matrix for 6 measurements
    	m_measurementNoiseCovarience_R.setIdentity();
    	m_measurementNoiseCovarience_R *= MEASUREMENT_NOISE_COVARIANCE_SCALE;

		// Set initial values for H. Update based on how measurements relate to the states
		//updateH();
    }

    Eigen::Quaterniond compute(
    		const Eigen::Vector3d& acc,
			const Eigen::Vector3d& gyro,
			const Eigen::Vector3d& magneto,
			const double& dt
			) override
    {
        // Prediction
        predict(gyro * DEGREE_TO_RAD, dt);

        Eigen::Vector<double, MeasureVectSize> tmp = computeMeasurementEstimate_h();

        // Update with accelerometer and magnetometer
        Eigen::Vector<double, MeasureVectSize> measurementVector(
        		acc(0),
				acc(1),
				acc(2),
				tmp(3),
				tmp(4),
				tmp(5)
				/*magneto(0),
				magneto(1),
				magneto(2)*/
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
    	// Compute state transition matrix and predict state estimate
    	m_stateTransition_A = computeStateTransitionMatrix(gyro, dt);
    	m_stateEstimate_X = m_stateTransition_A * m_stateEstimate_X;

    	// Normalize quaternion
    	// TODO: Create a function that may be overloaded if needed
    	m_stateEstimate_X.head<4>().normalize();


    	// Compute the Jacobian of the state transition model
    	Eigen::Matrix<double, StateVectSize, StateVectSize> F = computeJacobianStateTransitionModel();

		// Predict error covariance
		m_errorCovariance_P = F * m_errorCovariance_P * F.transpose() + m_noiseCovarience_Q;
    }

    /*
     * Correction step
     * Use accelerometer and magnetometer to correct the gyro estimation
     */
    void update(const Eigen::Vector<double, MeasureVectSize>& z)
    {
    	// Compute H based on the current state vector
    	// Jacobian of the measurement function matrix
    	Eigen::Matrix<double, MeasureVectSize, StateVectSize> H = computeJacobianMeasurementModel_H(z);

    	Eigen::Matrix<double, MeasureVectSize, MeasureVectSize> S = H * m_errorCovariance_P * H.transpose() + m_measurementNoiseCovarience_R;

    	// Kalman gain
    	Eigen::Matrix<double, StateVectSize, MeasureVectSize> K;
    	K = m_errorCovariance_P * H.transpose() * S.inverse();

    	// Compute expected measurement from accelerometer and magnetometer
    	Eigen::Vector<double, MeasureVectSize> h = computeMeasurementEstimate_h();

    	// Innovation (residual)
    	Eigen::Vector<double, MeasureVectSize> y = z - h;
    	//K.setIdentity();
    	// Update error state
    	/*Eigen::Vector4d quaternionCorrection = K.topRows<4>() * y;
    	// Convert correction term to a quaternion (small angle approximation)
    	Eigen::Quaterniond delta_q(1.0, 0.5 * quaternionCorrection(1), 0.5 * quaternionCorrection(2), 0.5 * quaternionCorrection(3));
    	// Update state quaternion with quaternion multiplication
		Eigen::Quaterniond current_q(m_stateEstimate_X(0), m_stateEstimate_X(1), m_stateEstimate_X(2), m_stateEstimate_X(3));
		Eigen::Quaterniond updated_q = current_q * delta_q;
		updated_q.normalize();
		// Update the state estimate quaternion part
		m_stateEstimate_X.head<4>() << updated_q.w(), updated_q.x(), updated_q.y(), updated_q.z();*/

		// Update state estimate
    	m_stateEstimate_X = m_stateEstimate_X + K * y;
    	// Normalize output quaternion
    	m_stateEstimate_X.head<4>().normalize();

    	// Update error covariance matrix
    	Eigen::Matrix<double, StateVectSize, StateVectSize> I;
    	I.setIdentity();
    	m_errorCovariance_P = (I - K * H) * m_errorCovariance_P;
    }

    Eigen::Quaterniond getEstimateAttitude() const
    {
    	// TODO: m_stateEstimate_X.head<4>()
        return Eigen::Quaterniond(m_stateEstimate_X(0), m_stateEstimate_X(1), m_stateEstimate_X(2), m_stateEstimate_X(3));
    }



    /*
     * Compute the state transition matrix
     * The quaternion part will integrate the gyro
     * The gyro offset part is just identity
     */
    // TODO: Remove parameters. Handle this in derived class
    virtual Eigen::Matrix<double, StateVectSize, StateVectSize> computeStateTransitionMatrix(const Eigen::Vector3d& gyro, const double& dt)
    {
    	// (Wx, Wy, Wz): angular rate

		//     1     -(dt/2)Wx  -(dt/2)Wy  -(dt/2)Wz
		// (dt/2)Wx       1      (dt/2)Wz  -(dt/2)Wy
		// (dt/2)Wy  -(dt/2)Wz       1      (dt/2)Wx
		// (dt/2)Wz   (dt/2)Wy  -(dt/2)Wx       1


		// Remove the estimated gyro offset (bias) from raw gyro
		double wx = gyro(0);// - m_stateEstimate_X(4); // XXX
		double wy = gyro(1);// - m_stateEstimate_X(5);
		double wz = gyro(2);// - m_stateEstimate_X(6);

		// Construct state transition matrix
		Eigen::Matrix4d Ac;
		Ac << 0.0, -wx, -wy, -wz,
				wx, 0.0, wz, -wy,
				wy, -wz, 0.0, wx,
				wz, wy, -wx, 0.0;
		Ac = 0.5 * Ac;

		Eigen::Matrix<double, 4, 4> A_quaternion = Eigen::Matrix4d::Identity() + Ac * dt;

		// Combine with identity for gyro offsets
		Eigen::Matrix<double, StateVectSize, StateVectSize> A;
		A.setIdentity();
		A.topLeftCorner<4, 4>() = A_quaternion;

		return A;
    }


    /*
     * Compute the Jaconian matrix of the state transition model.
     * Matrix of partial derivative.
     * It's purpose is to linearize non-linear systems
     */
    virtual Eigen::Matrix<double, StateVectSize, StateVectSize> computeJacobianStateTransitionModel()
    {
    	Eigen::Matrix<double, StateVectSize, StateVectSize> F;

    	F.setZero();
    	F.topLeftCorner<4, 4>() = m_stateTransition_A.topLeftCorner<4, 4>();  // State transition matrix for quaternion
    	//F.bottomRightCorner<3, 3>().setIdentity();  // Identity for gyro offstes  // XXX

    	return F;
    }


	/*
	 * Compute the measurement model h.
	 * h is not linear so we compute it's jacobian H.
	 * Matrix that map the measurement vector.
	 * Rows = Size of measurement vector
     * Columns = Size of state vector
	 */
    virtual Eigen::Matrix<double, MeasureVectSize, StateVectSize> computeJacobianMeasurementModel_H(const Eigen::Vector<double, MeasureVectSize>& z)
	{
		// ENU reference is used here.
		// ENU defines the (x, y, z) axis colinear to the geographical
		// East, North, and Up directions, respectively.

    	 // TODO: member variable in derived class
    	// Gravity reference vector
		const Eigen::Vector3d g(0.0, 0.0, 1.0);

		// TODO: measure value
		// TODO: member variable in derived class
		// Earth magnetic field reference vector
		const Eigen::Vector3d r(0.0, 1.0, 0.0);

		// Normalize accelerometer and magnetometer's reading
		Eigen::Vector3d a(z(0), z(1), z(2));
		a.normalize();
		Eigen::Vector3d m(z(3), z(4), z(5));
		m.normalize();

		const double g_x = g(0), g_y = g(1), g_z = g(2);
		const double r_x = r(0), r_y = r(1), r_z = r(2);
		const double q_w = m_stateEstimate_X(0);
		const double q_x = m_stateEstimate_X(1);
		const double q_y = m_stateEstimate_X(2);
		const double q_z = m_stateEstimate_X(3);

		// Create the jacobian of the measurement matrix
		// The topLeft<6, 4> map the quaternion, and the rest that is 0 map the gyro offset
		Eigen::Matrix<double, MeasureVectSize, StateVectSize> H;
		H.setZero();

		H << g_x*q_w + g_y*q_z - g_z*q_y, g_x*q_x - g_y*q_y - g_z*q_z, g_x*q_y + g_y*q_x + g_z*q_w, g_x*q_z - g_y*q_w + g_z*q_x, //0.0, 0.0, 0.0,  // XXX
		     -g_x*q_z + g_y*q_w + g_z*q_x, g_x*q_y + g_y*q_x - g_z*q_w, -g_x*q_w + g_y*q_z - g_z*q_y, g_x*q_x + g_y*q_y + g_z*q_z, //0.0, 0.0, 0.0,
		     g_x*q_y - g_y*q_x + g_z*q_w, g_x*q_z + g_y*q_w - g_z*q_x, g_x*q_w - g_y*q_z - g_z*q_y, g_x*q_x + g_y*q_y + g_z*q_z, //0.0, 0.0, 0.0,
		     r_x*q_w + r_y*q_z - r_z*q_y, r_x*q_x - r_y*q_y - r_z*q_z, r_x*q_y + r_y*q_x + r_z*q_w, r_x*q_z - r_y*q_w + r_z*q_x, //0.0, 0.0, 0.0,
		     -r_x*q_z + r_y*q_w + r_z*q_x, r_x*q_y + r_y*q_x - r_z*q_w, -r_x*q_w + r_y*q_z - r_z*q_y, r_x*q_x + r_y*q_y + r_z*q_z, //0.0, 0.0, 0.0,
		     r_x*q_y - r_y*q_x + r_z*q_w, r_x*q_z + r_y*q_w - r_z*q_x, r_x*q_w - r_y*q_z - r_z*q_y, r_x*q_x + r_y*q_y + r_z*q_z; //0.0, 0.0, 0.0;


		H = 2.0 * H;

		return H;
	}


    /*
     * Compute the expected accelerometer and magnetometer readings from current quaternion
     */
    virtual Eigen::Vector<double, MeasureVectSize> computeMeasurementEstimate_h()
    {
    	// Current quaternion
		Eigen::Quaterniond q(m_stateEstimate_X(0), m_stateEstimate_X(1), m_stateEstimate_X(2), m_stateEstimate_X(3));

		// Gravity vector in ENU frame // TODO duplicate code
		Eigen::Vector3d g(0.0, 0.0, 1.0); // TODO: member variable in derived class

		// Earth magnetic field vector in ENU frame TODO
		Eigen::Vector3d r(0.0, 1.0, 0.0); // TODO: member variable in derived class

		// Transform the gravity vector by the quaternion
		Eigen::Vector3d accExpected = q * g;

		// Transform the magnetic field vector by the quaternion
		Eigen::Vector3d magExpected = q * r;

		Eigen::Vector<double, MeasureVectSize> h;
		h << accExpected, magExpected;

		return h;
    }
};



