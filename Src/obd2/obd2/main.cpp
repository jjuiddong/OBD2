
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
	const bool isOpen = obd.Open(4, baudrates[0], &rcv, false);
	//if (!isOpen)
	//	return 0;

	while (g_isLoop)
	{
		obd.Query(cOBD2::PID_RPM);
		for (int i = 0; i < 100; ++i)
		{
			obd.Process(0.1f);
			Sleep(10);
		}
	}

	obd.Close();
}
