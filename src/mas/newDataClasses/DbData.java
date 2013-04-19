package newDataClasses;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

public class DbData implements IMongoSaveable {
	public String ip;
	public int port;
	public String name;
	
	public DbData(String ip, int port, String name){
		this.ip = ip;
		this.port = port;
		this.name = name;
	}

	/* (non-Javadoc)
	 * @see newDataClasses.IMongoSaveable#ToBasicDBObject()
	 */
	@Override
	public BasicDBObject ToBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start()
				.add("ip", ip)
				.add("port", port)
				.add("name", name).get();
	}

	/* (non-Javadoc)
	 * @see newDataClasses.IMongoSaveable#FromBasicDBObject(com.mongodb.BasicDBObject)
	 */
	@Override
	public void FromBasicDBObject(BasicDBObject object) {
		
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((ip == null) ? 0 : ip.hashCode());
		result = prime * result + ((name == null) ? 0 : name.hashCode());
		result = prime * result + port;
		return result;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DbData other = (DbData) obj;
		if (ip == null) {
			if (other.ip != null)
				return false;
		} else if (!ip.equals(other.ip))
			return false;
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		if (port != other.port)
			return false;
		return true;
	}

	/**
	 * @return the ip
	 */
	public String getIp() {
		return ip;
	}

	/**
	 * @param ip the ip to set
	 */
	public void setIp(String ip) {
		this.ip = ip;
	}

	/**
	 * @return the port
	 */
	public int getPort() {
		return port;
	}

	/**
	 * @param port the port to set
	 */
	public void setPort(int port) {
		this.port = port;
	}

	/**
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * @param name the name to set
	 */
	public void setName(String name) {
		this.name = name;
	}

	@Override
	public String toString() {
		return String.format("DbData [ip=%s, port=%s, name=%s]", ip, port, name);
	}
}
