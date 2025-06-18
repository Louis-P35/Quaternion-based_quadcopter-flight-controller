/*
 * mtf01.cpp
 *
 *  Created on: Jun 11, 2025
 *      Author: louis
 */

// Includes from project
#include "Sensors/mtf01.hpp"


/*
 * Init low pass filters
 */
bool Mtf01::init()
{
	constexpr float cutoffFrequency = 5.0f;

	// Init low pass filters
	m_lpfLidar.init(m_outputFrequency, cutoffFrequency);
	m_lpfVelX.init(m_outputFrequency, cutoffFrequency);
	m_lpfVelY.init(m_outputFrequency, cutoffFrequency);
}


/*
 *
 */
void Mtf01::readSensor()
{
	// Read data from sensor TODO
	float lidarRaw = 0.0f;
	float xVelRaw = 0.0f;
	float yVelRaw = 0.0f;

	// Apply low pass filters
	m_lidarDist = m_lpfLidar.apply(lidarRaw);
	m_xVelocity = m_lpfVelX.apply(xVelRaw);
	m_yVelocity = m_lpfVelY.apply(yVelRaw);
}


/*
 *
 */
float Mtf01::getLidarDist() const
{

}


/*
 *
 */
float Mtf01::getXVelocity() const
{

}


/*
 *
 */
float Mtf01::getYVelocity() const
{

}
