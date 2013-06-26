/**
 * @file ProductionEquipletMapper.java
 * @brief Class in which the equiplet can be assigned to/removed from a
 *        production step, same goes for the timeslots needed for the step.
 * @date Created: 02-04-2013
 * 
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
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

package rexos.mas.data;

import jade.core.AID;

import java.util.HashMap;

public class ProductionEquipletMapper{
	private HashMap<Integer, HashMap<AID, Long>> equipletList;

	public ProductionEquipletMapper(){
		equipletList = new HashMap<Integer, HashMap<AID, Long>>();
	}
	
	public HashMap<Integer, HashMap<AID, Long>> getHashMap(){
		return equipletList;
	}

	public void addProductionStep(int productionStepID){
		if (this.equipletList.containsKey(productionStepID) == false){
			this.equipletList.put(productionStepID, new HashMap<AID, Long>());
		} else{
			// DEBUG: Production step already exists
		}
	}

	public void removeProductionStep(Long productionStepID){
		if (this.equipletList.containsKey(productionStepID) == true){
			this.equipletList.remove(productionStepID);
		} else{
			// DEBUG: Production step doesn't exist
		}
	}

	public void addEquipletToProductionStep(Integer productionStepID,
			AID equipletID){
		this.addEquipletToProductionStep(productionStepID, equipletID, -1L);
	}

	public void addEquipletToProductionStep(Integer productionStepID,
			AID equipletID, Long timeslots){
		HashMap<AID, Long> tempEQHashmap = this.equipletList
				.get(productionStepID);
		tempEQHashmap.put(equipletID, timeslots);
		this.equipletList.put(productionStepID, tempEQHashmap);
	}

	public void removeEquipletFromProductionStep(Integer productionStepID,
			AID equipletID){
		HashMap<AID, Long> tempEQHashmap = this.equipletList
				.get(productionStepID);
		tempEQHashmap.remove(equipletID);
		this.equipletList.put(productionStepID, tempEQHashmap);
	}

	public HashMap<AID, Long> getEquipletsForProductionStep(int pA_id){
		// HashMap<AID, Long> s2 = this.equipletList.get(pA_id);
		return this.equipletList.get(pA_id);
	}

	public void setTimeSlotsForEquiplet(Integer productionStepID,
			AID equipletID, Long timeslots){
		HashMap<AID, Long> tempTimeslotForEQHashmap = this.equipletList
				.get(productionStepID);
		tempTimeslotForEQHashmap.put(equipletID, timeslots);
		this.equipletList.put(productionStepID, tempTimeslotForEQHashmap);
	}

	public long getTimeSlotsForEquiplet(int i, AID equipletID){
		return this.equipletList.get(i).get(equipletID);
	}
}
