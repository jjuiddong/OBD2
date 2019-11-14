//
// author: jjuiddong
//  2015-04-04
//
// CSerial Ŭ������ ��� �޾�, ReadStringUntil() �Լ��� ������ Ŭ������.
// Ư�� ���� (���๮��) ������ ������ �о� �� ��, ������� �����ϴ� cSerial Ŭ������
// ������� ����, ���۸� ȿ�������� �����ϸ鼭, ���ϴ� ����� �����ϴ� ������ �Ѵ�.
//
// �����۷� �����Ǿ, ���� ���Ҵ�, ���簡 �Ͼ�� �ʰ� �ߴ�.
// �� Ŭ������ ����� ���, ReadData()�Լ� ���, ReadStringUntil() �Լ��� ����Ѵ�.
//
//
// 2015-08-25
// ���� �ð� �̻󵿾� ch���ڰ� ������ ������, ���۸� �����Ѵ�.
//
// 2019-06-22
//	CircularQueue ����
//
// 2019-11-14, jjuiddong
//		- read buffer bugfix
//
//
#pragma once
#include "Serial.h"


namespace common
{

	class cBufferedSerial : public cSerial
	{
	public:
		cBufferedSerial();
		virtual ~cBufferedSerial();

		bool ReadStringUntil(const char ch, OUT char *out
			, OUT int &outLen, const int maxSize);
		void ClearBuffer();


	public:
		enum { MAX_BUFFERSIZE = 2048 };
		cCircularQueue<char, MAX_BUFFERSIZE> m_q;
	};

}
