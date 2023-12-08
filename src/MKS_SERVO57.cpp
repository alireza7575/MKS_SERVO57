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
	int send = sendMessage(stepperId, instruction::PING);
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

int MKS_SERVO57::getCurrentPosition(byte const &stepperId)
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
	return recieveEncoderPosition();
}

int MKS_SERVO57::recieveEncoderPosition()
{
	byte receivedBytes[10] = {0xFB, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30};
	size_t rd = port_->readBytes(receivedBytes, 10);
	int32_t carry = 0;
	for (int i = 3; i < 7; i++)
		carry = (carry << 8) | receivedBytes[i];
	uint16_t value = 0;
	for (int i = 7; i < 9; i++)
		value = (value << 8) | receivedBytes[i];
	return carry * 360 + value * 22 / 1000;
}

bool MKS_SERVO57::setTargetPosition(byte const &stepperId, byte const &dir, int const &speed, byte const &acc, uint32_t const &pulses)
{
	// Ensure parameters are within their valid ranges
	if (speed < 0 || speed > 1600 || acc > 32)
		return false;
	byte message[11];
	message[0] = HEADER;	// Header
	message[1] = stepperId; // Slave address
	message[2] = 0xFD;		// Function code for position mode
	// Speed and direction encoding
	message[3] = (dir << 7) | ((speed >> 4) & 0x0F);
	message[4] = speed & 0x0F;
	message[5] = acc; // Acceleration
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