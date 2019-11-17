
#include "stdafx.h"
#include "obd2.h"


cOBD2::cOBD2()
	: m_receiver(nullptr)
	, m_state(eState::DISCONNECT)
	, m_waitingTime(0)
	, m_queryCnt(0)
{
}

cOBD2::~cOBD2()
{
	Close();
}


// receiver: receive obd2 data
bool cOBD2::Open(const int comPort //= 2
	, const int baudRate //= 115200
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

	// no response, send another query
	m_waitingTime += deltaSeconds;
	if (!m_queryQ.empty() && (m_waitingTime > 1.f))
	{
		m_waitingTime = 0.f;
		while (!m_queryQ.empty())
			m_queryQ.pop();
		//Query(m_queryQ.front(), false); // send next query
		//m_queryQ.pop();
		//dbg::Logc(1, "rcv queue size = %d\n", m_ser.m_rcvQ.size());
		//dbg::Logc(1, "snd queue size = %d\n", m_ser.m_sndQ.size());
	}

	char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
	const uint readLen = m_ser.RecvData((BYTE*)buffer, sizeof(buffer));
	if (readLen <= 0)
		return true;

	if ((readLen == 1) && (buffer[0] == '\r'))
		return true;

	buffer[readLen] = NULL;
	if (readLen > 3)
	{
		m_rcvStr = buffer;
		if (m_isLog)
			Log(buffer);
	}

	//if (!strcpy_s(buffer, "NO DATA\r") && !m_queryQ.empty())
	//	m_ignorePIDs.insert((int)m_queryQ.front());

	m_waitingTime = 0.f;

	// parse pid data
	int pid = 0;
	char *p = buffer;
	char *data = nullptr;
	if (p = strstr(p, "41 "))
	{
		p += 3;
		pid = hex2uint8(p); // 2 byte
		data = p + 3;
	}

	// check query queue
	if (!m_queryQ.empty())
	{
		const bool isErr = (m_queryQ.front() != (ePID)pid);
		if (!isErr)
			m_queryQ.pop();

		if (!isErr && !m_queryQ.empty())
		{
			Query(m_queryQ.front(), false); // send next query
			m_queryQ.pop();
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


bool cOBD2::Query(const ePID pid
	, const bool isQueuing //= true
)
{
	if (!IsOpened())
		return false;

	if (m_ignorePIDs.end() != m_ignorePIDs.find((int)pid))
		return false; // ignore pid

	if (isQueuing)
	{
		if (m_queryQ.size() > MAX_QUEUE) // check max query size
			return false; // full queue, exit

		m_queryQ.push(pid);

		if (m_queryQ.size() >= 2) // serial busy? wait
			return true;
	}

	char cmd[8];
	sprintf_s(cmd, "%02X%02X\r", 1, (int)pid); //Service Mode 01
	m_ser.SendData((BYTE*)cmd, strlen(cmd));
	++m_queryCnt;
	return true;
}


// maybe connection check?
bool cOBD2::MemsInit()
{
	char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
	ZeroMemory(buffer, sizeof(buffer));
	//const uint readLen = SendCommand("ATTEMP\r", buffer, sizeof(buffer));
	//if ((readLen <= 0) || !strchr(buffer, '?'))
	//	return false;

	bool r = false; // result

	// set default
	//SendCommand("ATD\r", buffer, sizeof(buffer), "OK");

	// print id
	SendCommand("AT I\r", buffer, sizeof(buffer), "ELM327");

	// echo on/off
	SendCommand("AT E0\r", buffer, sizeof(buffer), "OK");
	//SendCommand("ATE0\r", buffer, sizeof(buffer), "OK");
	//SendCommand("ATE1\r", buffer, sizeof(buffer), "OK");

	// space on/off
	SendCommand("AT S0\r", buffer, sizeof(buffer), "OK");
	//SendCommand("ATS0\r", buffer, sizeof(buffer), "OK");
	//SendCommand("ATS1\r", buffer, sizeof(buffer), "OK");

	// protocol command
	// ISO 15765-4 CAN (29 bit ID, 500 kbaud)
	//SendCommand("ATSP7\r", buffer, sizeof(buffer), "OK");

	// Automatic protocol detection
	//SendCommand("ATSP0\r", buffer, sizeof(buffer), "OK");

	// Display current protocol using
	//SendCommand("ATDP\r", buffer, sizeof(buffer));

	// Steps for implementing a simple obd scanner in a micro controller using ELM327
	// http://dthoughts.com/blog/2014/11/06/obd-scanner-using-elm327/
	// ATZ -> ATSP0 -> 0100 -> recv 41 ~~ -> ATDP
	//r = SendCommand("ATZ\r", buffer, sizeof(buffer), "ELM327");
	//r = SendCommand("ATSP0\r", buffer, sizeof(buffer), "OK");
	//r = SendCommand("0100\r", buffer, sizeof(buffer), "41 ");
	//r = SendCommand("ATDP\r", buffer, sizeof(buffer));

	// factory reset
	//SendCommand("AT PP FF OFF\r", buffer, sizeof(buffer), "OK\r");
	//SendCommand("ATZ\r", buffer, sizeof(buffer), "ELM327");

	// set serial baudrate 115200
	// https://www.scantool.net/blog/switching-communication-baud-rate/
	// https://www.elmelectronics.com/wp-content/uploads/2016/06/AppNote04.pdf
	SendCommand("AT PP 0C SV 23\r", buffer, sizeof(buffer), "OK\r");

	// character echo setting
	//SendCommand("AT PP 09 FF \r", buffer, sizeof(buffer), "OK\r"); // echo off
	//SendCommand("AT PP 09 00 \r", buffer, sizeof(buffer), "OK\r"); // echo on

	// save setting
	SendCommand("AT PP ON\r", buffer, sizeof(buffer), "OK\r");

	// set default
	//r = SendCommand("ATD\r", buffer, sizeof(buffer), "OK");

	// reset all for update settings
	r = SendCommand("ATZ\r", buffer, sizeof(buffer), "ELM327");

	// https://www.sparkfun.com/datasheets/Widgets/ELM327_AT_Commands.pdf
	// SERIAL BAUDRATE 10400
	//SendCommand("IB 10\r", buffer, sizeof(buffer));

	// check connection
	r = SendCommand("ATTEMP\r", buffer, sizeof(buffer), "?");
	if (!r)
		return false;

	return true;
}


bool cOBD2::SendCommand(const char* cmd, char* buf, const uint bufsize
	, const string &untilStr //= ""
	, const uint timeout //= OBD_TIMEOUT_LONG
)
{
	if (!IsOpened())
		return 0;

	m_ser.SendData((BYTE*)cmd, strlen(cmd));
	uint readLen = 0;
	return ReceiveData(buf, bufsize, readLen, untilStr, 1000);
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
bool cOBD2::ReceiveData(char* buf, const uint bufsize
	, OUT uint &readLen
	, const string &untilStr
	, const uint timeout)
{
	if (!IsOpened())
		return false;

	uint t = 0;
	while (t < timeout)
	{
		const uint len = m_ser.RecvData((BYTE*)buf, bufsize);
		if (len == 0)
		{
			Sleep(10);
			t += 10;
			continue;
		}

		if ((len == 1) && (buf[0] == '\r'))
			continue;
		if ((len == 2) && (buf[0] == '>') && (buf[1] == '\r'))
			continue;

		if (bufsize > (uint)readLen)
			buf[readLen] = NULL;

		if (!untilStr.empty() && readLen > 0)
		{
			if (string::npos != string(buf).find(untilStr))
			{
				// maching string, success return
				readLen = len;
				return true;
			}
		}
		else if (untilStr.empty() && readLen > 0)
		{
			// no matching string, success return
			readLen = len;
			return true;
		}

		Sleep(10);
		t += 10;
	}

	// no matching or time out, fail return
	readLen = 0;
	return false;
}


bool cOBD2::Close()
{
	m_ser.Close();
	while (!m_queryQ.empty())
		m_queryQ.pop();
	m_state = eState::DISCONNECT;
	return true;
}
