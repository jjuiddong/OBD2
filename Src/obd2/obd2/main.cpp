
#include "stdafx.h"
#include "obd2.h"

class cOBDRecv : public iOBD2Receiver 
{
public:
	virtual void Recv(const int pid, const int data) override
	{
		std::cout << "pid:" << pid << ", data:" << data << std::endl;
	}
};

bool g_isLoop = true;
BOOL CtrlHandler(DWORD fdwCtrlType) {
	g_isLoop = false;
	return TRUE;
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
	const bool isOpen = obd.Open(6, baudrates[0], &rcv, true);
	if (!isOpen)
		return 0;

	int t = 0;
	while (g_isLoop && obd.IsOpened())
	{
		if (t > 100)
		{
			t = 0;
			obd.Query(cOBD2::PID_RPM);
			obd.Query(cOBD2::PID_SPEED);
		}

		Sleep(1);
		t++;
	}

	obd.Close();
}
