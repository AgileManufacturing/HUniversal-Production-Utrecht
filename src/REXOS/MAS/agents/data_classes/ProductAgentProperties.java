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
package agents.data_classes;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

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
	
	@Override
	public String toString() {
		   return "DataObject [product=" + _product + ", callback=" + _callback + "]";
	}
	
	
}
