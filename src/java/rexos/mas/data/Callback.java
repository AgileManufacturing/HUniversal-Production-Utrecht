/**
 *
 * Project: Product Agent
 *
 * Package: rexos.mas.data
 *
 * File: Callback.java
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
 *	Information used by the Product Agent to know which host (ip & port) to report to for status updates etc.
 *
 */

public class Callback{
	
	
	private String _host;
	
	private int _port;
	
	private int _reportLevel;
	
	
	
	public Callback() {
		
	}
	
	public String getHost() {
		return this._host;
	}
	
	public void setHost(String value) {
		this._host = value;
	}
	
	public int getPort() {
		return this._port;
	}
	
	public void setPort(int value) {
		this._port = value;
	}
	
	public int getReportLevel() {
		return this._reportLevel;
	}
	
	public void setReportLevel(int value) {
		this._reportLevel = value;
	}
	
	
	
	
	
	
}
