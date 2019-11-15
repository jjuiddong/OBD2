
#include "stdafx.h"
#include "obd2.h"

bool g_isLoop = true;
BOOL CtrlHandler(DWORD fdwCtrlType)
{
	g_isLoop = false;
	return TRUE;
}


class cOBDRecv : public iOBD2Receiver 
{
public:
	virtual void Recv(const int pid, const int data) override
	{
		std::cout << "pid:" << pid << ", data:" << data << std::endl;
	}
};

void WaitObd2(cOBD2 &obd)
{
	for (int i = 0; i < 1000; ++i)
	{
		obd.Process(0.001f);
		Sleep(1);
	}
}


int main()
{
	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE))
	{
		std::cout << "SetConsoleCtrlHandler failed, code : " << GetLastError() << std::endl;
		return -1;
	}

	cOBDRecv rcv;
	cOBD2 obd;
	const int baudrates[] = { 115200, 38400 };
	const bool isOpen = obd.Open(4, baudrates[0], &rcv, true);
	if (!isOpen)
		return 0;

	while (g_isLoop)
	{
		obd.Query(cOBD2::PID_RPM);
		WaitObd2(obd);
		obd.Query(cOBD2::PID_SPEED);
		WaitObd2(obd);
		obd.Query(cOBD2::PID_GPS_LATITUDE);
		WaitObd2(obd);
		obd.Query(cOBD2::PID_GPS_LONGITUDE);
		WaitObd2(obd);
	}

	obd.Close();
}
