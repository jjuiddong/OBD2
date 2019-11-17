
#include "stdafx.h"
#include "obd2.h"


cOBD2::cOBD2()
	: m_receiver(nullptr)
	, m_state(eState::Disconnect)
	, m_commState(eCommState::Send)
	, m_waitingTime(0)
	, m_queryCnt(0)
	, m_sleepMillis(10)
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

	m_comPort = comPort;
	m_baudRate = baudRate;
	m_isLog = isLog;
	m_receiver = receiver;

	m_state = eState::Connecting;
	m_commState = eCommState::Send;
	m_thread = std::thread(ThreadFunction, this);

	return true;
}


// push queryQ
bool cOBD2::Query(const ePID pid)
{
	if (!IsOpened())
		return false;

	common::AutoCSLock cs(m_cs);
	if (m_queryQ.size() > MAX_QUEUE) // check max query size
		return false; // full queue, exit

	m_queryQ.push(pid);
	return true;
}


// internal call function
// process odb2 communication
bool cOBD2::Process(const float deltaSeconds)
{
	const float recvTimeOut = 1.f; 

	if (!IsOpened())
		return false;
	if (m_queryQ.empty())
		return false;

	switch (m_commState)
	{
	case eCommState::Send:
	{
		m_cs.Lock();
		const ePID pid = (ePID)m_queryQ.front();
		m_cs.Unlock();

		char cmd[8];
		sprintf_s(cmd, "%02X%02X\r", 1, (int)pid); //Service Mode 01
		m_ser.SendData(cmd, strlen(cmd));
		++m_queryCnt;

		m_commState = eCommState::Recv;
		m_waitingTime = 0.f;
	}
	break;

	case eCommState::Recv:
	{
		m_waitingTime += deltaSeconds;
		if (m_waitingTime > recvTimeOut)
		{
			m_commState = eCommState::Send;
			if (!m_queryQ.empty())
			{
				m_cs.Lock();
				m_queryQ.pop();
				m_cs.Unlock();
			}
			break;
		}

		char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
		int readLen = 0;
		m_ser.ReadStringUntil('\r', buffer, readLen, sizeof(buffer));
		if (readLen <= 0)
			break;
		if ((readLen == 1) && (buffer[0] == '\r'))
			return true;

		buffer[readLen] = NULL;
		if (readLen > 3)
		{
			m_rcvStr = buffer;
			if (m_isLog)
				common::dbg::Logp(buffer);
		}

		m_waitingTime = 0.f;

		// parse pid data
		int pid = 0;
		char *data = nullptr;
		char *p = buffer;
		if (p = strstr(p, "41 "))
		{
			p += 3;
			pid = hex2uint8(p); // 2 byte
			data = p + 3;
		}

		// check query queue
		if (!m_queryQ.empty())
		{
			m_cs.Lock();
			const bool isErr = (m_queryQ.front() != (ePID)pid);
			m_queryQ.pop();
			m_cs.Unlock();
		}

		if (data)
		{
			int result = 0;
			if (NormalizeData((ePID)pid, data, result))
			{
				if (m_receiver) // call callback function
					m_receiver->Recv(pid, result);
			}
		}

		m_commState = eCommState::Send;
	}
	break;

	default:
		assert(0);
		break;
	}

	return true;
}


// maybe connection check?
bool cOBD2::MemsInit()
{
	char buffer[common::cBufferedSerial::MAX_BUFFERSIZE];
	ZeroMemory(buffer, sizeof(buffer));

	bool r = false; // result

	// set default
	//SendCommand("ATD\r", buffer, sizeof(buffer), "OK");

	// print id
	r = SendCommand("AT I\r", buffer, sizeof(buffer), "ELM327");

	// echo on/off
	//SendCommand("AT E0\r", buffer, sizeof(buffer), "OK");
	//SendCommand("ATE0\r", buffer, sizeof(buffer), "OK");
	//SendCommand("ATE1\r", buffer, sizeof(buffer), "OK");

	// space on/off
	//SendCommand("AT S0\r", buffer, sizeof(buffer), "OK");
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
	//SendCommand("AT PP 0C SV 23\r", buffer, sizeof(buffer), "OK\r");

	// character echo setting
	//SendCommand("AT PP 09 FF \r", buffer, sizeof(buffer), "OK\r"); // echo off
	//SendCommand("AT PP 09 00 \r", buffer, sizeof(buffer), "OK\r"); // echo on

	// save setting
	//SendCommand("AT PP ON\r", buffer, sizeof(buffer), "OK\r");

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


// blocking mode
// send, recv command
bool cOBD2::SendCommand(const char* cmd, char* buf, const uint bufsize
	, const string &untilStr //= ""
	, const uint timeout //= OBD_TIMEOUT_LONG
)
{
	if (!IsOpened())
		return 0;

	m_ser.SendData(cmd, strlen(cmd));
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
	using namespace std::chrono_literals;

	if (!IsOpened())
		return false;

	uint t = 0;
	while (t < timeout)
	{
		int len = 0;
		m_ser.ReadStringUntil('\r', buf, len, bufsize);
		if (len == 0)
		{
			std::this_thread::sleep_for(10ms);
			t += 11;
			continue;
		}

		if ((len == 1) && (buf[0] == '\r'))
			continue;
		if ((len == 2) && (buf[0] == '>') && (buf[1] == '\r'))
			continue;

		if (bufsize > (uint)len)
			buf[len] = NULL;

		if (!untilStr.empty() && len > 0)
		{
			if (string::npos != string(buf).find(untilStr))
			{
				// maching string, success return
				readLen = len;
				return true;
			}
		}
		else if (untilStr.empty() && len > 0)
		{
			// no matching string, success return
			readLen = len;
			return true;
		}

		std::this_thread::sleep_for(10ms);
		t += 11;
	}

	// no matching or time out, fail return
	readLen = 0;
	return false;
}


bool cOBD2::IsOpened() const
{
	return m_state != eState::Disconnect;
}


bool cOBD2::Close()
{
	m_state = eState::Disconnect;
	if (m_thread.joinable())
		m_thread.join();

	m_ser.Close();
	while (!m_queryQ.empty())
		m_queryQ.pop();

	return true;
}


// thread function
void cOBD2::ThreadFunction(cOBD2 *obd)
{
	obd->m_state = eState::Connecting;
	if (!obd->m_ser.Open(obd->m_comPort, obd->m_baudRate))
	{
		obd->m_state = eState::Disconnect;
		return;
	}

	if (!obd->MemsInit())
	{
		obd->m_ser.Close();
		obd->m_state = eState::Disconnect;
		return;
	}

	common::cTimer timer;
	timer.Create();

	obd->m_state = eState::Connect;
	while (obd->m_state == eState::Connect)
	{
		const double dt = timer.GetDeltaSeconds();
		obd->Process((float)dt);

		std::this_thread::sleep_for(
			std::chrono::duration<int, std::milli>(obd->m_sleepMillis));
	}
}
