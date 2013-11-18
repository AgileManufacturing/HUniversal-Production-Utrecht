/**
 * @file REXOS/MAS/libraries/schedule/data_classes/TimeSlotSynchronization.java
 * @brief Data class representing the synchronization of timeslots
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
/**
 * data class representing the synchronization of timeslots.
 * Timeslots will be synchronized in terms of milliseconds and the firsttimeslot in the grid. 
 * @author Roy Scheefhals 
 * 
 */
public class TimeSlotSynchronization {
	
	/**
	 * @var long firstTimeSlot
	 * 		The amount of milliseconds since the epoch ( 1970-01-01 )
	 * 		lets every schedule use this timeslot as the first timeslot
	 */
	private long firstTimeSlot;
	
	/**
	 * @var long timeSlotLength
	 * 		the length of the timeslot in milliseconds
	 */
	private long timeSlotLength;
	
	/**
	 * basic constructor for this class
	 * @param firstTimeSlot the firstTimeSlot in milliseconds since the epoch
	 * @param timeSlotLength the length of every timeslot in milliseconds
	 */
	public TimeSlotSynchronization(long firstTimeSlot, long timeSlotLength){
		this.firstTimeSlot = firstTimeSlot;
		this.timeSlotLength = timeSlotLength;		
	}
	
	/**
	 * getter for the first timeslot in milliseconds
	 * @return the firsttimeslot in milliseconds
	 */
	public long getFirstTimeSlot(){
		return firstTimeSlot;
	}
	
	/**
	 * getter for the length of a timeslot in milliseconds
	 * @return the timeslotlength in milliseconds
	 */
	public long getTimeSlotLength(){
		return timeSlotLength;
	}
	
	/**
	 * gets the timeslot with the current systemtime, synchronized with the firsttimeslot
	 * @return the currenttimeslot in de grid
	 */
	public long getCurrentTimeSlot(){
		return (System.currentTimeMillis() - firstTimeSlot) / timeSlotLength;
	}
}
