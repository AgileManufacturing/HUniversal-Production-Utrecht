/* 
 * File:   TimeData.h
 * Author: alexander-ubuntu
 *
 * Created on June 16, 2013, 3:36 PM
 */

#ifndef TIMEDATA_H
#define	TIMEDATA_H

namespace rexos_datatypes{
    class TimeData {
    public:
        TimeData();    
        int getDuration();
        void setDuration(int duration);
    private:
        int duration;
    };
}
#endif	/* TIMEDATA_H */

