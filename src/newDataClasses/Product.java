/**
 *
 * Project: product-agents
 *
 * Package: dataClasses
 *
 * File: Product.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package newDataClasses;


public class Product {
	
	private Production _production;

	public Product() {
		
	}
	
	public Product(Production production) throws Exception {
		if(production == null) throw new Exception("Production can't be null");
		setProduction(production);
	}

	/**
	 * @return the _production
	 */
	public Production getProduction() {
		return _production;
	}

	/**
	 * @param _production the _production to set
	 */
	public void setProduction(Production production) {
		this._production = production;
	}
	
	
	
}
