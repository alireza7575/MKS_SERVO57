#include "MKS_SERVO57.h"

void MKS_SERVO57::initialize(HardwareSerial *serialPort, long const &baudRate)
{
	port_ = serialPort;
	port_->begin(baudRate);
}

bool MKS_SERVO57::ping(byte const &stepperId)
{
	// Flush
	while (port_->read() != -1)
		;
	int send = sendMessage(stepperId, instruction::STEPPER_PING);
	if (send != 4)
		return -1;
	return reciveStepperStatus();
}

int MKS_SERVO57::sendMessage(byte const &stepperId, byte const &commandId)
{
	byte message[4];
	byte checksum = HEADER + stepperId + commandId;
	message[0] = HEADER;
	message[1] = stepperId;
	message[2] = commandId;
	message[3] = checksum & 0xFF;
	return port_->write(message, 4);
}

int MKS_SERVO57::reciveStepperStatus()
{
	int messageSize = 4 + sizeof(uint8_t);
	byte receivedBytes[messageSize];
	size_t rd = port_->readBytes(receivedBytes, messageSize);
	return receivedBytes[0] == 0xFB;
}

long MKS_SERVO57::getCurrentPosition(byte const &stepperId)
{
	// Flush
	while (port_->read() != -1)
		;
	int send = sendMessage(stepperId, instruction::GET_ENCODER_POS);
	if (send != 4)
	{
		Serial.println("Failed to send");
		return -1;
	}
	return recieveEncoderPosition(stepperId);
}

long MKS_SERVO57::recieveEncoderPosition(byte const &stepperId)
{
	byte receivedBytes[10] = {0xFB, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30};
	size_t bytesRead = port_->readBytes(receivedBytes, 10);
	if (bytesRead == 10 && receivedBytes[0] == 0xFB && receivedBytes[1] == stepperId)
	{
		int32_t carry = (int32_t)receivedBytes[3] << 24 | (int32_t)receivedBytes[4] << 16 | (int32_t)receivedBytes[5] << 8 | (int32_t)receivedBytes[6];
		uint16_t value = (uint16_t)receivedBytes[7] << 8 | (uint16_t)receivedBytes[8];
		return (carry * 0x3FFF) + value;
	}
	else
	{
		Serial.println("Invalid response from motor controller");
		return false;
	}
}

bool MKS_SERVO57::setTargetPosition(byte const &stepperId, byte const &direction, int const &speed, byte const &acceleration, uint32_t const &pulses)
{
	// Ensure parameters are within their valid ranges
	if (speed < 0 || speed > 1600 || acceleration > 32)
		return false;
	byte message[11];
	message[0] = HEADER;	// Header
	message[1] = stepperId; // Slave address
	message[2] = instruction::MOVE_SPEED_PULSES;		// Function code for position mode
	// Speed and direction encoding
	message[3] = (direction << 7) | ((speed >> 4) & 0x0F);
	message[4] = speed & 0x0F;
	message[5] = acceleration; // Acceleration
	// Pulses (4 bytes)
	message[6] = (pulses >> 24) & 0xFF;
	message[7] = (pulses >> 16) & 0xFF;
	message[8] = (pulses >> 8) & 0xFF;
	message[9] = pulses & 0xFF;
	message[10] = calculateChecksum(message, 10); // Checksum
	port_->write(message, sizeof(message)) == sizeof(message);
	int messageSize = 4 + sizeof(uint8_t);
	byte receivedBytes[messageSize];
	size_t rd = port_->readBytes(receivedBytes, messageSize);
	return receivedBytes[3] == 1;
	// Send the message
}

byte MKS_SERVO57::calculateChecksum(const byte *message, int length)
{
	byte checksum = 0;
	for (int i = 0; i < length; i++)
		checksum += message[i];
	return checksum & 0xFF;
}