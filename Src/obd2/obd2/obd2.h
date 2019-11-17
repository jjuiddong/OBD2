//
// 2019-11-14, jjuiddong
// OBD2 serial communication class
//		- serial multithreading
//
// reference
//		https://github.com/stanleyhuangyc/ArduinoOBD
//		- Written by Stanley Huang <stanleyhuangyc@gmail.com>
//
// OBD-II PIDs document
//		- https://en.wikipedia.org/wiki/OBD-II_PIDs
//
#pragma once

interface iOBD2Receiver {
	// override this method
	virtual void Recv(const int pid, const int data) = 0;
};


class cOBD2
{
public:
	// Mode 1 PIDs
	enum ePID {
		PID_ENGINE_LOAD = 0x04
		, PID_COOLANT_TEMP = 0x05
		, PID_SHORT_TERM_FUEL_TRIM_1 = 0x06
		, PID_LONG_TERM_FUEL_TRIM_1 = 0x07
		, PID_SHORT_TERM_FUEL_TRIM_2 = 0x08
		, PID_LONG_TERM_FUEL_TRIM_2 = 0x09
		, PID_FUEL_PRESSURE = 0x0A
		, PID_INTAKE_MAP = 0x0B
		, PID_RPM = 0x0C
		, PID_SPEED = 0x0D
		, PID_TIMING_ADVANCE = 0x0E
		, PID_INTAKE_TEMP = 0x0F
		, PID_MAF_FLOW = 0x10
		, PID_THROTTLE = 0x11
		, PID_AUX_INPUT = 0x1E
		, PID_RUNTIME = 0x1F
		, PID_DISTANCE_WITH_MIL = 0x21
		, PID_COMMANDED_EGR = 0x2C
		, PID_EGR_ERROR = 0x2D
		, PID_COMMANDED_EVAPORATIVE_PURGE = 0x2E
		, PID_FUEL_LEVEL = 0x2F
		, PID_WARMS_UPS = 0x30
		, PID_DISTANCE = 0x31
		, PID_EVAP_SYS_VAPOR_PRESSURE = 0x32
		, PID_BAROMETRIC = 0x33
		, PID_CATALYST_TEMP_B1S1 = 0x3C
		, PID_CATALYST_TEMP_B2S1 = 0x3D
		, PID_CATALYST_TEMP_B1S2 = 0x3E
		, PID_CATALYST_TEMP_B2S2 = 0x3F
		, PID_CONTROL_MODULE_VOLTAGE = 0x42
		, PID_ABSOLUTE_ENGINE_LOAD = 0x43
		, PID_AIR_FUEL_EQUIV_RATIO = 0x44
		, PID_RELATIVE_THROTTLE_POS = 0x45
		, PID_AMBIENT_TEMP = 0x46
		, PID_ABSOLUTE_THROTTLE_POS_B = 0x47
		, PID_ABSOLUTE_THROTTLE_POS_C = 0x48
		, PID_ACC_PEDAL_POS_D = 0x49
		, PID_ACC_PEDAL_POS_E = 0x4A
		, PID_ACC_PEDAL_POS_F = 0x4B
		, PID_COMMANDED_THROTTLE_ACTUATOR = 0x4C
		, PID_TIME_WITH_MIL = 0x4D
		, PID_TIME_SINCE_CODES_CLEARED = 0x4E
		, PID_ETHANOL_FUEL = 0x52
		, PID_FUEL_RAIL_PRESSURE = 0x59
		, PID_HYBRID_BATTERY_PERCENTAGE = 0x5B
		, PID_ENGINE_OIL_TEMP = 0x5C
		, PID_FUEL_INJECTION_TIMING = 0x5D
		, PID_ENGINE_FUEL_RATE = 0x5E
		, PID_ENGINE_TORQUE_DEMANDED = 0x61
		, PID_ENGINE_TORQUE_PERCENTAGE = 0x62
		, PID_ENGINE_REF_TORQUE = 0x63

		, PID_TRANSMISSION_GEAR = 0xA4
	};

	enum {
		OBD_TIMEOUT_SHORT = 1000 /* ms */
		, OBD_TIMEOUT_LONG = 5000 /* ms */
		, OBD_TIMEOUT_GPS = 200 /* ms */
	};

	cOBD2();
	virtual ~cOBD2();

	bool Open(const int comPort = 2, const int baudRate = 115200
		, iOBD2Receiver *receiver = nullptr
		, const bool isLog = false);
	bool Query(const ePID pid);
	bool Close();
	bool IsOpened() const;


protected:
	bool Process(const float deltaSeconds);
	bool MemsInit();
	bool SendCommand(const char* cmd, char* buf, const uint bufsize
		, const string &untilStr = ""
		, const uint timeout = OBD_TIMEOUT_LONG);
	bool NormalizeData(const ePID pid, char *data, OUT int &result);
	bool ReceiveData(char* buf, const uint bufsize
		, OUT uint &readLen
		, const string &untilStr
		, const uint timeout );
	static void ThreadFunction(cOBD2 *obd2);


public:
	enum {MAX_QUEUE = 200};
	enum class eState {Disconnect, Connecting, Connect};
	enum class eCommState {Send, Recv}; // communication state

	eState m_state;
	eCommState m_commState;
	int m_comPort;
	int m_baudRate;
	common::cBufferedSerial m_ser;

	queue<ePID> m_queryQ;
	iOBD2Receiver *m_receiver;
	string m_rcvStr;
	int m_queryCnt;
	bool m_isLog;
	float m_waitingTime;

	std::thread m_thread;
	common::CriticalSection m_cs;
	int m_sleepMillis; // default: 10ms
};

