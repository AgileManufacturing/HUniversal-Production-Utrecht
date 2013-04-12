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
package newDataClasses;

import jade.core.AID;

import java.util.ArrayList;
import java.util.HashMap;

public class ProductionEquipletMapper {
	
	private HashMap<Long, HashMap<AID, Long>> equipletList;
	
	public ProductionEquipletMapper() {
		equipletList = new HashMap<Long, HashMap<AID, Long>>();
	}
	
	public void addProductionStep(Long productionStepID) {
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
	
	public void addEquipletToProductionStep(Long productionStepID, AID equipletID) {
		this.addEquipletToProductionStep(productionStepID, equipletID, -1L);
	}
	
	public void addEquipletToProductionStep(Long productionStepID, AID equipletID, Long timeslots) {
		HashMap<AID, Long> tmp = this.equipletList.get(productionStepID);
		tmp.put(equipletID, timeslots);
		this.equipletList.put(productionStepID, tmp);
	}
	
	public void removeEquipletFromProductionStep(Long productionStepID, AID equipletID){
		HashMap<AID, Long> tmp = this.equipletList.get(productionStepID);
		tmp.remove(equipletID);
		this.equipletList.put(productionStepID, tmp);
	}
	
	public HashMap<AID, Long> getEquipletsForProductionStep(Long productionStepID) {
		return this.equipletList.get(productionStepID);
	}
	
	public void setTimeSlotsForEquplet(Long productionStepID, AID equipletID, Long timeslots) {
		HashMap<AID, Long> tmp = this.equipletList.get(productionStepID);
		tmp.put(equipletID, timeslots);
		this.equipletList.put(productionStepID, tmp);
	}
	
	public Long getTimeSlotsForEquiplet(Long productionStepID, AID equipletID) {
		return this.equipletList.get(productionStepID).get(equipletID);
	}
	

}
