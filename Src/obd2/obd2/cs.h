//
// 2019-11-15, jjuiddong
// critcal section
//
#pragma once


namespace common
{
	// Auto Lock, Unlock
	template<class T>
	class AutoLock
	{
	public:
		AutoLock(T& t) : m_t(t) { m_t.Lock(); }
		~AutoLock() { m_t.Unlock(); }
		T &m_t;
	};


	// Critical Section auto initial and delete
	class CriticalSection
	{
	public:
		CriticalSection();
		~CriticalSection();
		void Lock();
		void Unlock();
	protected:
		CRITICAL_SECTION m_cs;
	};

	inline CriticalSection::CriticalSection() {
		InitializeCriticalSection(&m_cs);
	}
	inline CriticalSection::~CriticalSection() {
		DeleteCriticalSection(&m_cs);
	}
	inline void CriticalSection::Lock() {
		EnterCriticalSection(&m_cs);
	}
	inline void CriticalSection::Unlock() {
		LeaveCriticalSection(&m_cs);
	}


	// auto critical section lock, unlock
	class AutoCSLock : public AutoLock<CriticalSection>
	{
	public:
		AutoCSLock(CriticalSection &cs) : AutoLock(cs) { }
	};
}
