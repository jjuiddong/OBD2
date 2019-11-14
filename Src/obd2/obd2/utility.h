//
// 2019-11-15, jjuiddong
// obd2 utility functions
//
// reference code
//		- https://github.com/stanleyhuangyc/ArduinoOBD
//		- Written by Stanley Huang <stanleyhuangyc@gmail.com>
//
#pragma once


inline uint16_t hex2uint16(const char *p)
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


inline byte hex2uint8(const char *p)
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


inline uint8_t getPercentageValue(char* data) {
	return (uint16_t)hex2uint8(data) * 100 / 255;
}

inline uint16_t getLargeValue(char* data) {
	return hex2uint16(data);
}

inline uint8_t getSmallValue(char* data) {
	return hex2uint8(data);
}

inline int16_t getTemperatureValue(char* data) {
	return (int)hex2uint8(data) - 40;
}


inline void Log(const char* fmt, ...)
{
	char buffer[256];
	ZeroMemory(buffer, sizeof(buffer));

	va_list args;
	va_start(args, fmt);
	vsnprintf_s(buffer, sizeof(buffer) - 1, _TRUNCATE, fmt, args);
	va_end(args);

	std::ofstream ofs("log.txt", std::ios::app);
	if (ofs.is_open())
		ofs << buffer;
}
