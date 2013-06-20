/**
 *
 * Project: Product Agent
 *
 * Package: rexos.mas.data
 *
 * File: ProductAgentProperties.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.data;

/**
 * @author Mike
 *
 */
public class ProductAgentProperties{
	
	private Product _product;
	
	//The host to connect to, represented as: IP or Hostname : Port number. eg. 127.0.0.1:253 or localhost:273
	private String _callbackHost;
	
	
	public ProductAgentProperties(){
		
	}
	
	public String getCallbackHost() {
		return this._callbackHost;
	}
	
	
	public void setCallbackHost(String value) {
		this._callbackHost = value;
	}
	
	public Product getProduct() {
		return this._product;
	}
	
	public void setProduct(Product value) {
		this._product = value;
	}
	
	
}
