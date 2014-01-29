/**
 * @file REXOS/MAS/libraries/schedule/data_classes/EquipletScheduleInformation.java
 * @brief Data object for the representation of freetimeslots for the EquipletSchedule. 
 * @date Created: 15 nov 2013
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

import java.io.Serializable;
import java.util.ArrayList;
import java.util.UUID;

/**
 * Data class for representation of FreeTimeSlots in a equiplet's schedule
 * Can be used to pass on to product agents for scheduling of steps
 * @author Roy Scheefhals
 *
 */
public class EquipletScheduleInformation implements Serializable {

	/**
	 * @var long serialVersionUID
	 * 		The serialization UID of this class
	 */
	private static final long serialVersionUID = 6144606966764813921L;

	/**
	 * @var ArrayList<FreeTimeSlot> freeTimeSlots
	 * 		Freetimeslots representing gaps in the schedule of the equiplet
	 */
	private ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
	
	/**
	 * @var ArrayList<TimeSlot> plannedTimeSlots 
	 * 		Freetimeslots representing gaps in the schedule of the equiplet
	 */
	private ArrayList<TimeSlot> plannedTimeSlots = new ArrayList<TimeSlot>();
	
	/**
	 * @var long infiniteFreeTimeSlot
	 * 		The folling start timeslot of the last scheduled step in an equipletschedule
	 */
	private long infiniteFreeTimeSlot;
	
	/**
	 * @var double load
	 * 		The load of an equiplet.
	 * 		value set from the EquipletSchedule will be between 0 and 1
	 */
	private double load;
	
	private boolean hasEquipletScheduleLock;
	
	private UUID equipletScheduleKey;
	
	/**
	 * Constructor of this dataclass
	 * @param freeTimeSlots The freetimeslots available for the equiplet
	 * @param infiniteFreeTimeSlot The timeslot after the last scheduled step
	 * @param load the load of the current EquipletSchedule
	 */
	public EquipletScheduleInformation(ArrayList<FreeTimeSlot> freeTimeSlots, long infiniteFreeTimeSlot, double load){
		this.freeTimeSlots = freeTimeSlots;
		this.infiniteFreeTimeSlot = infiniteFreeTimeSlot;
		this.equipletScheduleKey = null;
		this.hasEquipletScheduleLock = false;
	}
	
	
	
	/**
	 * gets the freetimeslots available in the equiplet's schedule
	 * @return the freetimeslots available
	 */
	public ArrayList<FreeTimeSlot> getFreeTimeSlots(){
		return freeTimeSlots;
	}
	
	/**
	 * gets the freetimeslots available in the equiplet's schedule that are equal or greather then the given dur
	 * @param dur the given duration to match the free timeslot.
	 * @return the freetimeslots available
	 */
	public ArrayList<FreeTimeSlot> getFreeTimeSlots(long dur) {
		ArrayList<FreeTimeSlot> resultList = new ArrayList<FreeTimeSlot>();
		
		for (FreeTimeSlot freeTimeSlot : freeTimeSlots) {
			if(freeTimeSlot.getDuration() >= dur){
				if(!plannedTimeSlots.contains(freeTimeSlot.getTimeSlot())){
					resultList.add(freeTimeSlot);
				}
			}
		}
		return resultList;
	}
	
	public void planTimeSlot(FreeTimeSlot freeTimeSlot){
		plannedTimeSlots.add(new TimeSlot(freeTimeSlot.getStartTimeSlot(), freeTimeSlot.getDuration()));
	} 
	
	/**
	 * gets the first timeslot available after the last scheduled step
	 * @return the timeslot 
	 */
	public long getinfiniteFreeTimeSlot(){
		return infiniteFreeTimeSlot;
	}
	
	/**
	 * gets the load of the equiplet in normalized form
	 * @return a normalized form of the load
	 */
	public double getLoad(){
		return load;
	}
	
	public boolean getIsEquipletScheduleLocked(){
		return hasEquipletScheduleLock;
	}
	
	public void setEquipletScheduleKey(UUID key){
		this.equipletScheduleKey = key;
		this.hasEquipletScheduleLock = true;
	}
	
	public UUID getEquipletScheduleKey(){
		return equipletScheduleKey;
	}
}
