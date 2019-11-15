
#include "stdafx.h"
#include "obd2.h"


cOBD2::cOBD2()
	: m_receiver(nullptr)
	, m_state(eState::DISCONNECT)
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
	, const bool isLog //= false
)
{
	Close();
	m_isLog = isLog;
	m_receiver = receiver;
	if (!m_ser.Open(comPort, baudRate, '\r'))
		return false;

	m_state = eState::CONNECTING;
	if (!MemsInit())
	{
		Close();
		return false;
	}

	return true;
}


// process odb2 communication
bool cOBD2::Process(const float deltaSeconds)
{
	if (!IsOpened())
		return false;

	char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
	const uint readLen = m_ser.RecvData((BYTE*)buffer, sizeof(buffer));
	if (readLen <= 0)
		return true;

	buffer[readLen] = NULL;
	if (m_isLog)
		Log(buffer);

	// parse pid data
	int pid = 0;
	char *p = buffer;
	char *data = nullptr;
	while (p = strstr(p, ">41"))
	{
		p += 3;
		BYTE curpid = hex2uint8(p); // 2 byte
		if (pid == 0) 
			pid = curpid;
		if (curpid == pid) 
		{
			data = p + 2;
			break;
		}
	}
	if (!data)
		return true;

	int result = 0;
	if (!NormalizeData((ePID)pid, data, result))
		return false;

	if (m_receiver) // call callback function
		m_receiver->Recv(pid, result);

	return true;
}


bool cOBD2::Query(const ePID pid)
{
	if (!IsOpened())
		return false;

	char cmd[8];
	sprintf_s(cmd, "%02X%02X\r", 1, (int)pid); //Service Mode 01
	m_ser.SendData((BYTE*)cmd, strlen(cmd));
	return true;
}


// maybe connection check?
bool cOBD2::MemsInit()
{
	char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
	ZeroMemory(buffer, sizeof(buffer));
	const uint readLen = SendCommand("ATTEMP\r", buffer, sizeof(buffer));
	if ((readLen > 0) && !strchr(buffer, '?'))
		return true;
	return false;
}


uint cOBD2::SendCommand(const char* cmd, char* buf, const uint bufsize
	, const uint timeout //= OBD_TIMEOUT_LONG
)
{
	if (!IsOpened())
		return 0;
	m_ser.SendData((BYTE*)cmd, strlen(cmd));
	return ReceiveData(buf, bufsize, 1000);
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


// read data until timeout
// timeout : milliseconds unit
uint cOBD2::ReceiveData(char* buf, const uint bufsize, const uint timeout)
{
	if (!IsOpened())
		return false;

	int readLen = 0;
	uint t = 0;
	while (t < timeout)
	{
		const uint readLen = m_ser.RecvData((BYTE*)buf, bufsize);
		if (readLen > 0)
			break;
		Sleep(10);
		t += 10;
	}
	return (uint)readLen;
}


bool cOBD2::Close()
{
	m_ser.Close();
	m_state = eState::DISCONNECT;
	return true;
}
