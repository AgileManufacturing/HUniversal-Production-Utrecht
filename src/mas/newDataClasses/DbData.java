package newDataClasses;

import java.io.Serializable;

public class DbData implements Serializable{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	public String ip;
	public int port;
	public String name;
	
	public DbData(String ip, int port, String name){
		this.ip = ip;
		this.port = port;
		this.name = name;
	}

	@Override
	public String toString() {
		return String.format("DbData [ip=%s, port=%s, name=%s]", ip, port, name);
	}
}
