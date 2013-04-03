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

public class Production {
	
	private ProductionStep[] _productionSteps;
	
	public Production() {
		
	}
	
	public Production(ProductionStep[] productionSteps) throws Exception {
		if(productionSteps == null) throw new Exception("Production steps can't be null");
		this._productionSteps = productionSteps;
	}
	
	public ProductionStep[] getProductionSteps() {
		return _productionSteps;
	}
	
}
