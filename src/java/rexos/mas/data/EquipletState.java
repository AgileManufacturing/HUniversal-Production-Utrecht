/**
 * @file rexos/mas/data/EquipletState.java
 * @brief 
 * @date Created: 24 jun. 2013
 *
 * @author Peter Bonnema
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/
package rexos.mas.data;

public enum EquipletState {
	/**
	 * @var EquipletState SAFE
	 *      safe state of the equiplet
	 */
	SAFE(0),
	/**
	 * @var EquipletState SETUP
	 *      setup state of the equiplet
	 */
	SETUP(1),
	/**
	 * @var EquipletState SHUTDOWN
	 *      shutdown state of the equiplet
	 */
	SHUTDOWN(2),
	/**
	 * @var EquipletState STANDBY
	 *      standby state of the equiplet
	 */
	STANDBY(3),
	/**
	 * @var EquipletState START
	 *      start state of the equiplet
	 */
	START(4),
	/**
	 * @var EquipletState STOP
	 *      stop state of the equiplet
	 */
	STOP(5),
	/**
	 * @var EquipletState NORMAL
	 *      normal state of the equiplet
	 */
	NORMAL(6),
	/**
	 * @var EquipletState NOSTATE
	 *      nostate state of the equiplet
	 */
	NOSTATE(7);

	/**
	 * @var int value
	 *      The value of the state
	 */
	private int value;

	/**
	 * constructor for the state
	 * 
	 * @param value The value of the state
	 */
	private EquipletState(int value) {
		this.value = value;
	}
	
	/**
	 * @return the value
	 */
	public int getValue() {
		return value;
	}

	public static EquipletState getState(int value) {
		for(EquipletState state : EquipletState.values()) {
			if(state.value == value) {
				return state;
			}
		}
		return null;
	}
}
