//
// precompiled header
//
#pragma once

#include <iostream>
#include <fstream>
#include <thread>
#include <queue>
#include <windows.h>

#ifndef IN
	#define IN
#endif

#ifndef OUT
	#define OUT
#endif

#ifndef INOUT
	#define INOUT
#endif

#define RETV(exp,val)	{if((exp)) return val; }


typedef unsigned int uint;

#include "cs.h"
#include "circularqueue.h"
#include "serial.h"
#include "BufferedSerial.h"
#include "SerialAsync.h"
#include "utility.h"
#include "obd2.h"

