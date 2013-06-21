/**
 *
 * Project: Dummy Product Agent
 *
 * Package: newDataClasses
 *
 * File: ProductionEquipletMapper.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.data;

import jade.core.AID;

import java.util.HashMap;

public class ProductionEquipletMapper {
	
	private HashMap<Integer, HashMap<AID, Long>> equipletList;
	
	public ProductionEquipletMapper() {
		equipletList = new HashMap<>();
	}
		
	public HashMap<Integer, HashMap<AID, Long>> getHashMap(){
		return equipletList;
	}
	
	public void addProductionStep(int productionStepID) {
		if(this.equipletList.containsKey(productionStepID) == false) {
			this.equipletList.put(productionStepID, new HashMap<AID, Long>());
		} else {
			//DEBUG: Production step already exists
		}
	}
	
	public void removeProductionStep(Long productionStepID) {
		if(this.equipletList.containsKey(productionStepID) == true) {
			this.equipletList.remove(productionStepID);
		} else {
			//DEBUG: Production step doesn't exist
		}
	}
	
	public void addEquipletToProductionStep(Integer productionStepID, AID equipletID) {
		this.addEquipletToProductionStep(productionStepID, equipletID, -1L);
	}
	
	public void addEquipletToProductionStep(Integer productionStepID, AID equipletID, Long timeslots) {
		HashMap<AID, Long> tmp = this.equipletList.get(productionStepID);
		tmp.put(equipletID, timeslots);
		this.equipletList.put(productionStepID, tmp);
	}
	
	public void removeEquipletFromProductionStep(Integer productionStepID, AID equipletID){
		HashMap<AID, Long> tmp = this.equipletList.get(productionStepID);
		tmp.remove(equipletID);
		this.equipletList.put(productionStepID, tmp);
	}
	
	public HashMap<AID, Long> getEquipletsForProductionStep(int pA_id) {
		return this.equipletList.get(pA_id);
	}
	
	public void setTimeSlotsForEquplet(Integer productionStepID, AID equipletID, Long timeslots) {
		HashMap<AID, Long> tmp = this.equipletList.get(productionStepID);
		tmp.put(equipletID, timeslots);
		this.equipletList.put(productionStepID, tmp);
	}
	
	public Long getTimeSlotsForEquiplet(int i, AID equipletID) {
		return this.equipletList.get(i).get(equipletID);
	}
	

}
