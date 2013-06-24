/**
 *
 * Project: ProductAgent
 *
 * Package: rexos.mas.data
 *
 * File: GUIMessage.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.data;

import com.google.gson.annotations.SerializedName;

/**
 * @author Mike
 *
 */
public class GUIMessage {
	
	@SerializedName("error")
	private boolean _error;
	
	@SerializedName("message")
	private String _message;
	
	@SerializedName("payload")
	private String _payload;
	
	
	public void setError(boolean value) {
		this._error = value;
	}
	
	public boolean getError() {
		return this._error;
	}
	
	public void setMessage(String value) {
		this._message = value;
	}
	
	public String getMessage() {
		return this._message;
	}
	
	public void setPayload(String value) {
		this._payload = value;
	}
	
	public String getPayload() {
		return this._payload;
	}
	
	@Override
	public String toString() {
	   return "DataObject [error=" + _error + ", message=" + _message + ", payload="
		+ _payload + "]";
	}

}
