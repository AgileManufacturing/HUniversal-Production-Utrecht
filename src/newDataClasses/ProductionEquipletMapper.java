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

public class ProductionEquipletMapper extends Mapper<Long, AID> {
	
	public ProductionEquipletMapper() {
		
	}
	
	public void addProductionStep(Long productionStepID) {
		if(this._items.containsKey(productionStepID) == false) {
			this._items.put(productionStepID, new ArrayList<AID>());
		}
	}
	
	public void removeProductionStep(Long productionStepID) {
		if(this._items.containsKey(productionStepID) == true) {
			this._items.remove(productionStepID);
		}
	}
	
	public void addEquipletToProductionStep(Long productionStepID, AID equipletID) {
		ArrayList<AID> tmp = this._items.get(productionStepID);
		tmp.add(equipletID);
		this._items.put(productionStepID, tmp);
	}
	
	public ArrayList<AID> getEquipletsForProductionStep(Long productionStepID) {
		return this._items.get(productionStepID);
	}
	

}
