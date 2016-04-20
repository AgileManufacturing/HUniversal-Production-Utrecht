#ifndef _BASETIMER_H_
#define _BASETIMER_H_

#include <string>
#include <fstream>
#include <map>

typedef std::map<std::string, unsigned long long> valuelist;

/** This is the main class of the library.
This class is responsible for all the timing. It uses the High-Performance
counters of Windows.
*/
class BaseTimer {
private:
	valuelist __values;

	unsigned long long GetPerformanceTicks();
	unsigned long long GetPerformanceTicksInSecond();

	unsigned long long  __start;
	unsigned long long  __stop;
	unsigned long long __nFreq;
public:
	BaseTimer();

	bool start();
	bool stop();
	void reset();

	unsigned long long elapsedMicroSeconds();
	unsigned long long elapsedMilliSeconds();
	double             elapsedSeconds();

	void store(std::string label);
	bool save(std::string filename);
};

#endif  // _BASETIMER_H_