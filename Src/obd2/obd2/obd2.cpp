
#include "stdafx.h"
#include "obd2.h"

// https://github.com/stanleyhuangyc/ArduinoOBD
uint16_t hex2uint16(const char *p)
{
	char c = *p;
	uint16_t i = 0;
	for (char n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		}
		else if (c >= 'a' && c <= 'f') {
			c -= 39;
		}
		else if (c == ' ') {
			continue;
		}
		else if (c < '0' || c > '9') {
			break;
		}
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

byte hex2uint8(const char *p)
{
	byte c1 = *p;
	byte c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >= 'a' && c1 <= 'f')
		c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
		c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}

uint8_t getPercentageValue(char* data) {
	return (uint16_t)hex2uint8(data) * 100 / 255;
}
uint16_t getLargeValue(char* data) {
	return hex2uint16(data);
}
uint8_t getSmallValue(char* data) {
	return hex2uint8(data);
}
int16_t getTemperatureValue(char* data) {
	return (int)hex2uint8(data) - 40;
}


cOBD2::cOBD2()
	: m_receiver(nullptr)
{
}

cOBD2::~cOBD2()
{
	Close();
}


// receiver: receive obd2 data
bool cOBD2::Open(const int comPort //= 2
	, const int baudRate //= 9600
	, iOBD2Receiver *receiver //=nullptr
)
{
	Close();

	if (!m_ser.Open(comPort, baudRate))
		return false;

	MemsInit();

	return true;
}


// 
bool cOBD2::Process(const float deltaSeconds)
{
	if (!IsOpened())
		return false;

	int readLen = 0;
	char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
	m_ser.ReadStringUntil('\r', buffer, readLen, sizeof(buffer));
	if (readLen <= 0)
		return true;

	// parse pid data
	buffer[readLen] = NULL;
	int pid = 0;
	char *p = buffer;
	char *data = nullptr;
	while (p = strstr(p, "410"))
	{
		p += 3;
		BYTE curpid = hex2uint8(p);
		if (pid == 0) 
			pid = curpid;
		if (curpid == pid) 
		{
			p += 2;
			if (*p == ' ')
			{
				data = p + 1;
				break;
			}
		}
	}

	if (!data)
		return true;

	int result = 0;
	if (!NormalizeData((ePID)pid, data, result))
		return false;

	if (m_receiver)
		m_receiver->Recv(pid, result);

	return true;
}


bool cOBD2::Query(const ePID pid)
{
	if (!IsOpened())
		return false;

	char cmd[8];
	sprintf_s(cmd, "%02X%02X\r", 1, (int)pid);
	m_ser.SendData(cmd, strlen(cmd));
	return true;
}


bool cOBD2::MemsInit()
{
	char buf[16];
	return SendCommand("ATTEMP\r", buf, sizeof(buf)) > 0 && !strchr(buf, '?');
}


BYTE cOBD2::SendCommand(const char* cmd, char* buf, byte bufsize
	, int timeout //= OBD_TIMEOUT_LONG
)
{
	if (!IsOpened())
		return false;

	m_ser.SendData(cmd, strlen(cmd));
	//dataIdleLoop();
	//return receive(buf, bufsize, timeout);
	return 0;
}


bool cOBD2::NormalizeData(const ePID pid, char *data
	, OUT int &result)
{
	//int result;
	switch (pid) 
	{
	case PID_RPM:
	case PID_EVAP_SYS_VAPOR_PRESSURE: // kPa
		result = getLargeValue(data) >> 2;
		break;
	case PID_FUEL_PRESSURE: // kPa
		result = getSmallValue(data) * 3;
		break;
	case PID_COOLANT_TEMP:
	case PID_INTAKE_TEMP:
	case PID_AMBIENT_TEMP:
	case PID_ENGINE_OIL_TEMP:
		result = getTemperatureValue(data);
		break;
	case PID_THROTTLE:
	case PID_COMMANDED_EGR:
	case PID_COMMANDED_EVAPORATIVE_PURGE:
	case PID_FUEL_LEVEL:
	case PID_RELATIVE_THROTTLE_POS:
	case PID_ABSOLUTE_THROTTLE_POS_B:
	case PID_ABSOLUTE_THROTTLE_POS_C:
	case PID_ACC_PEDAL_POS_D:
	case PID_ACC_PEDAL_POS_E:
	case PID_ACC_PEDAL_POS_F:
	case PID_COMMANDED_THROTTLE_ACTUATOR:
	case PID_ENGINE_LOAD:
	case PID_ABSOLUTE_ENGINE_LOAD:
	case PID_ETHANOL_FUEL:
	case PID_HYBRID_BATTERY_PERCENTAGE:
		result = getPercentageValue(data);
		break;
	case PID_MAF_FLOW: // grams/sec
		result = getLargeValue(data) / 100;
		break;
	case PID_TIMING_ADVANCE:
		result = (int)(getSmallValue(data) / 2) - 64;
		break;
	case PID_DISTANCE: // km
	case PID_DISTANCE_WITH_MIL: // km
	case PID_TIME_WITH_MIL: // minute
	case PID_TIME_SINCE_CODES_CLEARED: // minute
	case PID_RUNTIME: // second
	case PID_FUEL_RAIL_PRESSURE: // kPa
	case PID_ENGINE_REF_TORQUE: // Nm
		result = getLargeValue(data);
		break;
	case PID_CONTROL_MODULE_VOLTAGE: // V
		result = getLargeValue(data) / 1000;
		break;
	case PID_ENGINE_FUEL_RATE: // L/h
		result = getLargeValue(data) / 20;
		break;
	case PID_ENGINE_TORQUE_DEMANDED: // %
	case PID_ENGINE_TORQUE_PERCENTAGE: // %
		result = (int)getSmallValue(data) - 125;
		break;
	case PID_SHORT_TERM_FUEL_TRIM_1:
	case PID_LONG_TERM_FUEL_TRIM_1:
	case PID_SHORT_TERM_FUEL_TRIM_2:
	case PID_LONG_TERM_FUEL_TRIM_2:
	case PID_EGR_ERROR:
		result = ((int)getSmallValue(data) - 128) * 100 / 128;
		break;
	case PID_FUEL_INJECTION_TIMING:
		result = ((int32_t)getLargeValue(data) - 26880) / 128;
		break;
	case PID_CATALYST_TEMP_B1S1:
	case PID_CATALYST_TEMP_B2S1:
	case PID_CATALYST_TEMP_B1S2:
	case PID_CATALYST_TEMP_B2S2:
		result = getLargeValue(data) / 10 - 40;
		break;
	case PID_AIR_FUEL_EQUIV_RATIO: // 0~200
		result = (long)getLargeValue(data) * 200 / 65536;
		break;
	default:
		result = getSmallValue(data);
	}
	return true;
}


bool cOBD2::Close()
{
	m_ser.ClearBuffer();
	m_ser.Close();
	return true;
}
