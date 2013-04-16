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

import java.util.List;

import newDataClasses.sqldatadase.sqliteDatabase;


public class Product {
	
	private Production _production;
	private ProductLog log; 

	/**
	 * @return the log
	 */
	public ProductLog getLog() {
		return log;
	}

	public Product() {
		
	}
	
	public Product(Production production, String aid) throws Exception {
		if(production == null) throw new Exception("Production can't be null");
		setProduction(production);
		log = new ProductLog(false, true, new sqliteDatabase(aid));
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
	
	public void add(List<LogMessage> msg){
		log.add(msg);
	}
}
