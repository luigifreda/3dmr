#ifndef ZTIMER_H
#define ZTIMER_H

#include <string>
#include <map>
#include <ctime>
#include <fstream>

#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>

boost::mutex ztimer_mtx;

#define DEFAULT_LOG_FILE_NAME "cputime.log"

class ZTimer
{
private:
	timespec start;
public:
	ZTimer()
	{
		clock_gettime(CLOCK_REALTIME, &start);
	}

	void measure(timespec *pdiff, double *pfdiff)
	{
		timespec end, diff;

		// take end time
		clock_gettime(CLOCK_REALTIME, &end);

		// compute difference
		if ((end.tv_nsec - start.tv_nsec) < 0) {
			diff.tv_sec = end.tv_sec - start.tv_sec - 1;
			diff.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
		} else {
			diff.tv_sec = end.tv_sec - start.tv_sec;
			diff.tv_nsec = end.tv_nsec - start.tv_nsec;
		}

		// write to output if not NULL
		if(pdiff) memcpy(pfdiff, &diff, sizeof(timespec));
		if(pfdiff) *pfdiff = double(diff.tv_sec) + 1e-9 * diff.tv_nsec;

		// start <- end
		memcpy(&start, &end, sizeof(timespec));
	}

	double measure()
	{
		double ret;
		measure(NULL, &ret);
		return ret;
	}

	double measureAndLog(std::string log_key, std::string log_file_name = DEFAULT_LOG_FILE_NAME)
	{
		return measureAndLog(log_key, 0, log_file_name);
	}

	double measureAndLog(std::string log_key, long i, std::string log_file_name = DEFAULT_LOG_FILE_NAME)
	{
		timespec t;
		double ft;
		measure(&t, &ft);

		ztimer_mtx.lock();
		std::ofstream log(log_file_name.c_str(), std::ios_base::app | std::ios_base::out);
		log << boost::format("%s\t%ld\t%.10f") % log_key % i % ft << std::endl;
		log.flush();
		log.close();
		ztimer_mtx.unlock();

		return ft;
	}
};

#endif // ZTIMER_H
