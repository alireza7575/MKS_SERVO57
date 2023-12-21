/// \file MKS_SERVO57.h
/// \brief Provides an interface for controlling MKS SERVO57 stepper motors via serial commands.
///
/// \details  The MKS_SERVO57 class encapsulates the necessary methods to communicate and control
///           MKS SERVO57 stepper motors. It offers functionality to initialize the motor interface
///           with a given serial port and baud rate, send a ping to check motor connectivity, retrieve
///           the current encoder position, check if the motor is in motion, and set a target position
///           with specified direction, speed, acceleration, and number of pulses. The class handles
///           the underlying serial communication protocol, providing a user-friendly API for motor
///           operations.
#ifndef MKS_SERVO57_h
#define MKS_SERVO57_h

#include <Arduino.h>

namespace instruction
{
	const byte GET_ENCODER_POS = 0x30;
	const byte STEPPER_PING = 0x3A;
	byte const MOVE_SPEED_PULSES = 0xFD;
};

class MKS_SERVO57
{
public:
	/// @brief Initializes the serial communication with the stepper motor.
	/// @param serialPort Pointer to a HardwareSerial object for communication.
	/// @param baudRate Baud rate for the serial communication.
	///        			Defaults to 38400 if not specified.
	void initialize(HardwareSerial *serialPort = nullptr, long const &baudRate = 38400);

	/// @brief Checks if the stepper motor with the specified ID is connected and responsive.
	/// @param stepperId The unique identifier for the stepper motor.
	/// @return Returns true if the motor responds to the ping, false otherwise.
	bool ping(byte const &stepperId);

	/// @brief Retrieves the current position of the stepper motor in pulses.
	/// @param stepperId The ID of the stepper motor whose position is to be retrieved.
	/// @return Returns the current position of the stepper motor as a long integer value.
	long getCurrentPosition(byte const &stepperId);

	/// @brief Commands the stepper motor to move to a target position.
	/// @param stepperId The unique identifier for the stepper motor.
	/// @param direction The direction of the motor movement (0 for one direction, 1 for the opposite).
	/// @param speed The speed at which the motor should move.
	/// @param acceleration The acceleration profile for the motor movement.
	/// @param pulses The number of pulses corresponding to the target position.
	/// @return Returns true if the command is successfully sent, false otherwise.
	bool setTargetPosition(byte const &stepperId, byte const &direction, int const &speed, byte const &acceleration, uint32_t const &pulses);

private:
	HardwareSerial *port_;
	byte const HEADER = 0xFA;

	int sendMessage(byte const &stepperId, byte const &commandID);
	int reciveStepperStatus();
	long recieveEncoderPosition(byte const &stepperId);
	byte calculateChecksum(const byte *message, int length);
};

#endif
