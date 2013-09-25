package gatewayserver.data;
import com.google.gson.annotations.SerializedName;

/**
 *
 * Project: GatewayServer
 *
 * Package: 
 *
 * File: Command.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */

/**
 * @author Mike
 *
 */
public class Command {

	private int id;
	private String command;
	
	@SerializedName("payload")
	private String payload;
	
	public Command() {
		
	}
	
	public Command(int id, String comand, String payload) {
		this.id = id;
		this.command = comand;
		this.payload = payload;
	}
	
	/**
	 * @return the id
	 */
	public int getId() {
		return id;
	}

	/**
	 * @param id the id to set
	 */
	public void setId(int id) {
		this.id = id;
	}

	/**
	 * @return the command
	 */
	public String getCommand() {
		return command;
	}

	/**
	 * @param command the command to set
	 */
	public void setCommand(String command) {
		this.command = command;
	}

	/**
	 * @return the product
	 */
	public String getPayload() {
		return payload;
	}

	/**
	 * @param product the product to set
	 */
	public void setPayload(String product) {
		this.payload = product;
	}

	@Override
	public String toString() {
	   return "DataObject [id=" + id + ", command=" + command + ", payload="
		+ payload + "]";
	}
	
}
