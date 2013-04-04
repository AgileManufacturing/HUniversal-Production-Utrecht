/**
 *
 * Project: product-agents
 *
 * Package: dataClasses
 *
 * File: Production.java
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

public class Production {
	
	private ProductionStep[] _productionSteps;
	private HashMap<Long, ArrayList<AID>> _plausibleEquiplets;
	
	public Production() {
		
	}
	
	public Production(ProductionStep[] productionSteps) throws Exception {
		if(productionSteps == null) throw new Exception("Production steps can't be null");
		this._productionSteps = productionSteps;
		
		this._plausibleEquiplets = new HashMap<Long, ArrayList<AID>>();
		
		for(ProductionStep p : this._productionSteps){
			_plausibleEquiplets.put(p.getId(), new ArrayList<AID>());
		}
	}
	
	public ProductionStep[] getProductionSteps() {
		return _productionSteps;
	}
	
	public void addEquipletToProductionStep(long stepId, AID equipletId){
		ArrayList<AID> tmp = _plausibleEquiplets.get(stepId);
		tmp.add(equipletId);
	}
	
	public void removeEquipletFromProductionStep(long stepId, AID equipletId){
		ArrayList<AID> tmp = _plausibleEquiplets.get(stepId);
		tmp.remove(tmp.indexOf(equipletId));
	}
	
	public ArrayList<AID> getPlausibleEquiplets(long stepId){
		return _plausibleEquiplets.get(stepId);
	}
	
}
