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
	
	private ProductionEquipletMapper _prodletmap;

	
	
	public Production() {
		_prodletmap = new ProductionEquipletMapper();
	}
	
	public Production(ProductionStep[] productionSteps) throws Exception {
		this();
		if(productionSteps == null) throw new Exception("Production steps can't be null");
		this._productionSteps = productionSteps;
		
		for(ProductionStep p : this._productionSteps){
			this._prodletmap.addProductionStep(p.getId());
		}
	}
	
	public ProductionStep[] getProductionSteps() {
		return _productionSteps;
	}
	
	public void setProductionEquipletMapping(ProductionEquipletMapper prodLetMap) throws Exception {
		if(prodLetMap == null) throw new Exception("mapping cant be null");
		this._prodletmap = prodLetMap;
	}
	
	public ProductionEquipletMapper getProductionEquipletMapping() {
		return this._prodletmap;
	}
	
}
