#include "mrm-node.h"
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_node::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_node::Mrm_node(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "Node", maxNumberOfBoards, ID_MRM_NODE, 1) {
	readings = new std::vector<uint16_t[MRM_NODE_ANALOG_COUNT]>(maxNumberOfBoards);
	switches = new std::vector<bool[MRM_NODE_SWITCHES_COUNT]>(maxNumberOfBoards);
	servoDegrees = new std::vector<uint16_t[MRM_NODE_SERVO_COUNT]>(maxNumberOfBoards);
	
	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({COMMAND_NODE_SENDING_SENSORS_1_TO_3, 	"Send 1-3"});
		commandNamesSpecific->insert({COMMAND_NODE_SENDING_SENSORS_4_TO_6, 	"Send 4-6"});
		commandNamesSpecific->insert({COMMAND_NODE_SENDING_SENSORS_7_TO_9, 	"Send 7-9"});
		commandNamesSpecific->insert({COMMAND_NODE_SWITCH_ON,				"Switch on"});
		commandNamesSpecific->insert({COMMAND_NODE_SERVO_SET,				"Servo set"});
	}
}

Mrm_node::~Mrm_node()
{
}

/** Add a mrm-node sensor
@param deviceName - device's name
*/
void Mrm_node::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_NODE0_IN;
		canOut = CAN_ID_NODE0_OUT;
		break;
	case 1:
		canIn = CAN_ID_NODE1_IN;
		canOut = CAN_ID_NODE1_OUT;
		break;
	case 2:
		canIn = CAN_ID_NODE2_IN;
		canOut = CAN_ID_NODE3_OUT;
		break;
	case 3:
		canIn = CAN_ID_NODE3_IN;
		canOut = CAN_ID_NODE4_OUT;
		break;
	case 4:
		canIn = CAN_ID_NODE4_IN;
		canOut = CAN_ID_NODE4_OUT;
		break;
	case 5:
		canIn = CAN_ID_NODE5_IN;
		canOut = CAN_ID_NODE5_OUT;
		break;
	case 6:
		canIn = CAN_ID_NODE6_IN;
		canOut = CAN_ID_NODE6_OUT;
		break;
	case 7:
		canIn = CAN_ID_NODE7_IN;
		canOut = CAN_ID_NODE7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}

	for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
		(*switches)[nextFree][i] = 0;

	for (uint8_t i = 0; i < MRM_NODE_SERVO_COUNT; i++)
		(*servoDegrees)[nextFree][i] = 0xFFFF;

	SensorBoard::add(deviceName, canIn, canOut);
}

std::string Mrm_node::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_node::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				bool any = false;
				uint8_t startIndex = 0;
				switch (message.data[0]) {
				case COMMAND_NODE_SENDING_SENSORS_1_TO_3:
					startIndex = 0;
					any = true;
					break;
				case COMMAND_NODE_SENDING_SENSORS_4_TO_6:
					startIndex = 3;
					any = true;
					break;
				case COMMAND_NODE_SENDING_SENSORS_7_TO_9:
					startIndex = 6;
					any = true;
					device.lastReadingsMs = millis();
					break;
				case COMMAND_NODE_SWITCH_ON: {
					uint8_t switchNumber = message.data[1] >> 1;
					if (switchNumber > 4) {
						strcpy(errorMessage, "No switch");
						return false;
					}
					(*switches)[device.number][switchNumber] = message.data[1] & 1;
					device.lastReadingsMs = millis();
				}
										   break;
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
				}

				if (any)
					for (uint8_t i = 0; i <= 2; i++)
						(*readings)[device.number][startIndex + i] = (message.data[2 * i + 1] << 8) | message.data[2 * i + 2];
			}
			return true;
		}
	return false;
}

/** Analog readings
@param receiverNumberInSensor - single IR transistor in mrm-ref-can
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_node::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || receiverNumberInSensor > MRM_NODE_ANALOG_COUNT) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName.c_str(), deviceNumber);
		return 0;
	}
	if (started(deviceNumber))
		return (*readings)[deviceNumber][receiverNumberInSensor];
	else
		return 0;
	// return (*readings)[deviceNumber][receiverNumberInSensor];
}

/** Print all readings in a line
*/
void Mrm_node::readingsPrint() {
	print("Ref. array:");
	for (Device& device: devices){
		for (uint8_t irNo = 0; irNo < MRM_NODE_ANALOG_COUNT; irNo++)
			print(" %3i", (*readings)[device.number][irNo]);
	}
}

/** Test servos
*/
void Mrm_node::servoTest() {
	static uint64_t lastMs = 0;

	if (millis() - lastMs > 100) {
		for (uint8_t deg = 0; deg <= 180; deg += 5) {
			for (Device& device: devices) {
				if (device.alive) {
					for (uint8_t servoNumber = 0; servoNumber < MRM_NODE_SERVO_COUNT; servoNumber++)
						servoWrite(servoNumber, deg, device.number);
				}
			}
			print("%i deg.\n\r", deg);
			delay(100);
		}
		lastMs = millis();
	}
}

/** Move servo
@servoNumber - 0 - 2
@degrees - 0 - 180 degrees
@deviceNumber - mrm-node id
*/
void Mrm_node::servoWrite(uint8_t servoNumber, uint16_t degrees, uint8_t deviceNumber) {
	if (servoNumber >= MRM_NODE_SERVO_COUNT) {
		strcpy(errorMessage, "Servo not found");
		return;
	}
	if (degrees != (*servoDegrees)[deviceNumber][servoNumber]) {
		canData[0] = COMMAND_NODE_SERVO_SET;
		canData[1] = servoNumber;
		canData[2] = degrees >> 8;
		canData[3] = degrees & 0xFF;
		(*servoDegrees)[deviceNumber][servoNumber] = degrees;

		messageSend(canData, 4, deviceNumber);
	}
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_node::started(uint8_t deviceNumber) {
	if (millis() - devices[deviceNumber].lastReadingsMs > MRM_NODE_INACTIVITY_ALLOWED_MS || devices[deviceNumber].lastReadingsMs == 0) {
		//print("Start mrm-node%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&devices[deviceNumber], 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - devices[deviceNumber].lastReadingsMs < 100) {
					//print("Lidar confirmed\n\r"); 
					return true;
				}
				delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), deviceNumber);
		return false;
	}
	else
		return true;
}

/** Read digital
@param switchNumber
@deviceNumber - mrm-node id
@return
*/
bool Mrm_node::switchRead(uint8_t switchNumber, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || switchNumber >= MRM_NODE_SWITCHES_COUNT) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName.c_str(), deviceNumber);
		return false;
	}
	return (*switches)[deviceNumber][switchNumber];
}


/**Test
*/
void Mrm_node::test()
{
	static uint64_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (Device& device: devices) {
			if (device.alive) {
				if (pass++)
					print("| ");
				print("An:");
				for (uint8_t i = 0; i < MRM_NODE_ANALOG_COUNT; i++)
					print("%i ", (*readings)[device.number][i]);
				print("Di:");
				for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
					print("%i ", (*switches)[device.number][i]);
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}
