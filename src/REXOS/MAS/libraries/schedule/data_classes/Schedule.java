/**
 * @file src/REXOS/MAS/libraries/schedule/data_classes/Schedule.java
 * @brief abstract class for a schedule in REXOS MAS system
 * @date Created: 14 nov 2013
 * 
 * @author Roy Scheefhals
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/


package libraries.schedule.data_classes;

import java.util.UUID;

/**
 * Abstract class to create a schedule implementation
 * the extending class will need to use this class's scheduleLock functions
 * for proper atomic operation on the schedules
 * @author Roy Scheefhals
 *
 */
public abstract class Schedule{
	
	/**
	 * @var String PLANNING_NAME
	 * 		The prefix name of the planning part of the whole schedule
	 */
	protected final String PLANNING_NAME= "Planning";
	
	/**
	 * @var String FREETIMESLOT_NAME
	 * 		The prefix name of the freetimeslots part of the whole schedule
	 */
	protected final String FREETIMESLOT_NAME= "FreeTimeSlot";
	
	/**
	 * @var Sring REALTIMESCHEDULE_NAME
	 * 		The prefix name of the realtime schedule part of the whole schedule
	 */
	protected final String REALTIMESCHEDULE_NAME = "RealtimeSchedule";
	
	/**
	 * @var String scheduleHostName
	 * 		The hosting location of the schedule
	 */
	protected String scheduleHostName;
	/**
	 * @var int schedulePort
	 * 		the port of the hosting location of the schedule
	 */
	protected int schedulePort;
		
	/**
	 * @var ScheduleLock scheduleLock
	 * 		The used schedulelock for the extending class
	 */
	private ScheduleLock scheduleLock;
	
	/**
	 * Constructor that needs to be called by te extending class.
	 * Cannot be used normally
	 * @param scheduleHostName the host location of the schedule
	 * @param schedulePort the port of the host of the schedule
	 */
	protected Schedule( String scheduleHostName, int schedulePort){
		this.scheduleHostName = scheduleHostName;
		this.schedulePort = schedulePort;
		
		this.scheduleLock = new ScheduleLock();
	}
	
	/**
	 * Checks the lock if the key used is the right one
	 * @param key the key used to check the lock
	 * @throws ScheduleAccessException when the given key of the lock isnt the right one, 
	 * a ScheduleAccessException will be thrown
	 */
	public void checkLock(UUID key) throws ScheduleAccessException{
		if (! scheduleLock.isCurrentOwner(key)){
			throw new ScheduleAccessException("not owner of lock");
		}
	}
	
	/**
	 * gets a new key for the current lock ( if it is already unlocked )
	 * @return the new key for this lock, but null if the lock is already locked
	 */
	public UUID getScheduleLock(){
		return scheduleLock.acquireScheduleLock();
	}
	
	/**
	 * tries to release the lock with the given key
	 * @param key the key given to release the lock
	 * @return true if the lock has been released or was already released, false otherwise
	 */
	public boolean releaseScheduleLock(UUID key){
		return scheduleLock.releaseLock(key);
	}
	
	
}
