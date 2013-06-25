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

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

/**
 * @author Mike
 *
 *	Information used by the Product Agent to know which host (ip & port) to report to for status updates etc.
 *
 */

public class Callback{
	
	@Expose
	@SerializedName("host")
	private String _host;
	
	@Expose
	@SerializedName("port")
	private int _port;
	
	@Expose
	@SerializedName("reportLevel")
	private int _reportLevel;
	
	@Expose
	@SerializedName("protocol")
	private String _protocol;
	
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
	
	public String getProtocol() {
		return this._protocol;
	}
	
	public void setProtocol(String protocol) {
		this._protocol = protocol;
	}
	
	@Override
	public String toString() {
		   return "DataObject [host=" + _host + ", port=" + _port + ", reportLevel="+_reportLevel+", protocol="+_protocol+"]";
	}
	
}
