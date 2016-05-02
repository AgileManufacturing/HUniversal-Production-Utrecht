#include "basetimer.h"
#include <windows.h>
#include <iostream>


/**
The main constructor of the BaseTimer class.
This initializes everything to zero.
*/
BaseTimer::BaseTimer() {
	__nFreq = GetPerformanceTicksInSecond();
	reset();
}

/** This will start a timing. If a timing has already been started, the function will return false.
@returns true is timing has started, false is a timing has already been started.
*/
bool BaseTimer::start() {
	if (__start == 0) {
		__start = GetPerformanceTicks();
		return true;
	}
	else return false;
}

/** This will stop a timing. If a timing was not started, the function will return false.
@returns true is timing has stopped, false is a timing was not started.
*/
bool BaseTimer::stop() {
	if (__start != 0) {
		__stop = GetPerformanceTicks();
		return true;
	}
	else return false;
}

/** This resets all the data (except the stored values) to zero. After this call you
can perform a new timing by calling the start() and stop() functions.
*/
void BaseTimer::reset() {
	__start = 0;
	__stop = 0;
}

/** This function will calculate the duration of the last timing.
@return The time in microseconds between the start() and the stop()
*/
unsigned long long BaseTimer::elapsedMicroSeconds() {
	unsigned long long nTicks = __stop - __start;
	return 1000000 * nTicks / __nFreq;
}

/** This function will calculate the duration of the last timing.
@return The time in milliseconds between the start() and the stop()
*/
unsigned long long BaseTimer::elapsedMilliSeconds() {
	unsigned long long nTicks = __stop - __start;
	return 1000 * nTicks / __nFreq;
}

/** This function will calculate the duration of the last timing.
@return The time in seconds between the start() and the stop()
*/
double BaseTimer::elapsedSeconds() {
	unsigned long long nTicks = __stop - __start;
	return static_cast<double>(nTicks) / static_cast<double>(__nFreq);
}

unsigned long long BaseTimer::GetPerformanceTicks() {
	LARGE_INTEGER nValue;
	::QueryPerformanceCounter(&nValue);
	return nValue.QuadPart;
}

unsigned long long BaseTimer::GetPerformanceTicksInSecond() {
	LARGE_INTEGER nFreq;
	::QueryPerformanceFrequency(&nFreq);
	return nFreq.QuadPart;
}

/** The timer is able to store multiple timings. To store the last timing
you call this function with a label of the timing as the argument.
If the label already exists it will overwrite that value, the label should
therefore be unique
@param label The unique label for the last timing
*/
void BaseTimer::store(std::string label) {
	__values[label] = elapsedMicroSeconds();
}

/** This will save all the stored timings into a single file with the filename
specified in the paramater.
@param filename The filename of the file where the stored values will be saved.
*/
bool BaseTimer::save(std::string filename) {
	std::ofstream output(filename.c_str());
	if (output.is_open()) {
		for (valuelist::iterator i = __values.begin(); i != __values.end(); ++i) {
			output << i->first << ";" << i->second << std::endl;
		}
		return true;
	}
	else return false;
}