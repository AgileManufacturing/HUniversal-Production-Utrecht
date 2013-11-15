/**
 * @file rexos/mas/equiplet_agent/ScheduleLock.java
 * @brief Class used as a lock for the schedule of an equiplet
 * @date Created: 2013-11-08
 * 
 * @author Roy Scheefhals
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package libraries.schedule.data_classes;

import java.util.UUID;

public class ScheduleLock {
	
	/**
	 * @var boolean scheduleLock
	 * 		if the lock is applied or not
	 */
	private boolean scheduleLock;
	
	/**
	 * @var AID lockGivenTo
	 * 		the AID of an agent the lock is given to
	 */
	private UUID currentKey;
	
	/**
	 * Basic constructor.
	 */
	public ScheduleLock(){
		scheduleLock = false;
		currentKey = null;
	}
	
	/**
	 * Function for acquiring a lock.
	 * @return null if the schedule could not be locked, or a new UUID will be given as key to this lock
	 */
	public UUID acquireScheduleLock(){
		if ( scheduleLock ){
			return null;
		}
		currentKey = UUID.randomUUID();
		scheduleLock = true;
		return currentKey;
	}
	
	/**
	 * Releases the lock, But requires the id of the asking agent 
	 * @param agent The AID of the agent asking the release
	 * @return True if the lock is succesfully released or was already released, 
	 * 		   false if the asker is not the current owner 
	 */
	public boolean releaseLock(UUID key){
		if (!scheduleLock){
			return true;
		}
		if (key.equals(currentKey)){
			scheduleLock = false;
			currentKey = null;
			return true;
		}
		return false;
	}
	
	/**
	 * checks the current owner of the lock
	 * @param key the key used to check the ownership
	 * @return true if the given key is the right one
	 */
	public boolean isCurrentOwner(UUID key){
		if (currentKey.equals(key)){
			return true;
		}
		return false;
	}
}
