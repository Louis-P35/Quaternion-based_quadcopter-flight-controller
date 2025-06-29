/*
 * mtf01.cpp
 *
 *  Created on: Jun 11, 2025
 *      Author: louis
 */

// Includes from project
#include "Sensors/mtf01.hpp"
#include "logManager.hpp"

// Includes from HAL
#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart4;
extern uint8_t mtf01BufCopy[];


// Mavlink packet structure:
// [STX=0xFE][LEN][SEQ][SYSID][COMPID][MSGID][PAYLOAD][CRC_L][CRC_H]
// STX : Start byte (0xFE for MAVLink v1).
// LEN : Payload lenght (1–255 bytes).
// SEQ : Sequence number (for packet loss detection).
// SYSID : System ID (ex. : 1 for the drone).
// COMPID : Component ID(ex. : 100 for a sensor).
// MSGID : Message ID(ex. : 100 for OPTICAL_FLOW).
// PAYLOAD : Data (ex. : flowX, flowY, height for OPTICAL_FLOW).
// CRC : 16 bits checksum (CRC16-X25) to validate the buffer integrity.


/*
 * MAVLink CRC calculation (based on ArduPilot)
 */
uint16_t MavlinkProtocole::calculateCrc(const uint8_t* pBuffer, const size_t& len, const uint8_t& crcExtra) const
{
	uint16_t crc = 0xFFFF;

	auto acc = [&](uint8_t b){
		uint8_t tmp = b ^ (crc & 0xFF);
		tmp ^= (tmp << 4);
		crc = (crc >> 8) ^ (uint16_t(tmp) << 8)
						^ (uint16_t(tmp) << 3)
						^ (uint16_t(tmp) >> 4);
	};

	for (size_t i = 0; i < len; ++i)
	{
		acc(pBuffer[i]);
	}
	acc(crcExtra);

	return crc;
}


/*
 * Handle the rx frame with a state machine, as it is received byte per byte.
 */
void MavlinkProtocole::handleByte(const uint8_t& byte)
{
    switch (m_parseState)
    {
        case ParseState::WAITING_FOR_STX:
            if (byte == MAVLINK_STX_V1)
            {
                m_rxBuffer[0] = byte;
                m_rxIndex = 1;
                m_parseState = ParseState::LEN;
            }
            break;

        case ParseState::LEN:
        	m_packet.len = byte;
            m_payloadLength = byte;
            m_rxBuffer[m_rxIndex++] = byte;
            m_parseState = ParseState::SEQ;
            break;

        case ParseState::SEQ:
        	m_packet.seq = byte;
        	m_rxBuffer[m_rxIndex++] = byte;
        	m_parseState = ParseState::SYSID;
            break;

        case ParseState::SYSID:
        	m_packet.sysid = byte;
        	m_rxBuffer[m_rxIndex++] = byte;
        	m_parseState = ParseState::COMPID;
            break;

        case ParseState::COMPID:
        	m_packet.compid = byte;
        	m_rxBuffer[m_rxIndex++] = byte;
        	m_parseState = ParseState::MSGID;
            break;

        case ParseState::MSGID:
        	m_packet.msgid = byte;
        	m_rxBuffer[m_rxIndex++] = byte;
        	m_parseState = ParseState::PAYLOAD;
            break;

        case ParseState::PAYLOAD:
            if (m_rxIndex < m_payloadLength + 6)
            {
            	// +6 for STX, LEN, SEQ, SYSID, COMPID, MSGID
            	m_packet.payload[m_rxIndex - 6] = byte;
            	m_rxBuffer[m_rxIndex++] = byte;
            }
            if (m_rxIndex >= m_payloadLength + 6)
            {
            	m_parseState = ParseState::CRC_L;
            }
            break;

        case ParseState::CRC_L:
        	m_packet.crc = byte;
        	m_rxBuffer[m_rxIndex++] = byte;
        	m_parseState = ParseState::CRC_H;
            break;

        case ParseState::CRC_H:
        	m_packet.crc |= (uint16_t)byte << 8;
        	m_rxBuffer[m_rxIndex++] = byte;

        	/*for (int i = 0; i < m_rxIndex; ++i)
        	{
        		LogManager::getInstance().serialPrint((int)m_rxBuffer[i]);
        		LogManager::getInstance().serialPrint(" ");
        	}
        	LogManager::getInstance().serialPrint("\r\n");*/

            // Validate and parse packet
            if (m_rxIndex == m_payloadLength + 8)
            {
            	uint8_t extra = 0;
            	if (m_packet.msgid == OPTICAL_FLOW_MSG_ID)
            	{
            		extra = OPTICAL_FLOW_CRC_EXTRA;
            	}
            	else if (m_packet.msgid == DISTANCE_MSG_ID)
            	{
            		extra = DISTANCE_SENSOR_CRC_EXTRA;
            	}

            	// Full packet: STX + LEN + SEQ + SYSID + COMPID + MSGID + PAYLOAD + CRC
                uint16_t expectedCrc = calculateCrc(&m_rxBuffer[1], m_payloadLength + 5, extra);
                if (expectedCrc == m_packet.crc)
                {
                	m_dataValid = decodeBuffer();
                }
                else
                {
                	m_dataValid = false;
                }
            }
            else
            {
            	m_dataValid = false;
            }

            // Reset state
            m_parseState = ParseState::WAITING_FOR_STX;
            m_rxIndex = 0;
            break;

        default:
        	// Reset state
        	m_parseState = ParseState::WAITING_FOR_STX;
        	break;
    }
}


/*
 *	Decode the read Mavlink packets.
 *	Can be Optical Flow packet or Lidar packet.
 */
bool MavlinkProtocole::decodeBuffer()
{
    switch (m_packet.msgid)
    {
    // Optical Flow (26 bytes)
    case OPTICAL_FLOW_MSG_ID:
        if (m_payloadLength != 26)
        {
        	return false;
        }

        {
            const uint8_t* p = m_packet.payload.data();

            uint64_t time_usec = 0;
            for (int i = 7; i >= 0; --i)
            {
            	time_usec = (time_usec << 8) | p[i];
            }

            uint8_t qual   =  p[25];
            int16_t flowX =  p[20] | (p[21] << 8);
            int16_t flowY =  p[22] | (p[23] << 8);

            /*LogManager::getInstance().serialPrint((int)flowX);
            LogManager::getInstance().serialPrint("\t");
            LogManager::getInstance().serialPrint((int)flowY);*/
            /*LogManager::getInstance().serialPrint((int)qual);
            LogManager::getInstance().serialPrint("\r\n");*/

            m_flowRawX = flowX;
            m_flowRawY = flowY;
            m_quality = qual;
        }
        return true;

    // Lidar (14 bytes)
    case DISTANCE_MSG_ID:
        if (m_payloadLength != 14)
        {
        	return false;
        }

        {
            const uint8_t* p = m_packet.payload.data();

            uint32_t time_ms   =  p[0] | (p[1] << 8) | (p[2]  << 16) | (p[3]  << 24);

            uint16_t current = p[8] | (p[9] << 8);      // Current distance (in cm)
            uint16_t minDist = p[4] | (p[5] << 8);      // Min distance the sensor is capable (in cm)
            uint16_t maxDist = p[6] | (p[7] << 8);      // Max distance the sensor is capable (in cm)

            if (current != 0xFFFF) // If dist is valid
            {
                m_height = 0.01f * current;  // Meter conversion
            }
        }
        return true;

    default:
        return false;   // Unhandled message
    }
}


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

	return true;
}


/*
 *
 */
void Mtf01::readSensor()
{
	// Read data from sensor TODO next
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
	// TODO next
	return 0.0f;
}


/*
 *
 */
float Mtf01::getXVelocity() const
{
	// TODO next
	return 0.0f;
}


/*
 *
 */
float Mtf01::getYVelocity() const
{
	// TODO next
	return 0.0f;
}
