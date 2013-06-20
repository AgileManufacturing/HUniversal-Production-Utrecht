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
	private Callback _callback;
	
	
	public ProductAgentProperties(){
		
	}
	
	public Callback getCallback() {
		return this._callback;
	}
	
	public void setCallback(Callback value) {
		this._callback = value;
	}
	
	public Product getProduct() {
		return this._product;
	}
	
	public void setProduct(Product value) {
		this._product = value;
	}
	
	
}
