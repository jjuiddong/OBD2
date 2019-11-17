//
// precompiled header
//
#pragma once

#include <iostream>
#include <fstream>
#include <thread>
#include <queue>
#include <set>
#include <assert.h>
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
#define RETV2(exp,val)	{if((exp)) {assert(0); return val;} }


typedef unsigned int uint;

using std::string;
using std::set;
using std::queue;


#include "cs.h"
#include "circularqueue.h"
#include "serial.h"
#include "BufferedSerial.h"
#include "timer.h"
#include "utility.h"
#include "obd2.h"
