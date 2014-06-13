package generic;

/**
 * 
 * @author Tommas Bakker
 *
 */
public class Service {
	private String name;
	
	public Service(String name){
		this.name = name;
	}
	
	public String getName(){
		return this.name;
	}
	
	public boolean compareName(String serviceName){
		if(this.name.equals(serviceName)){
			return true;
		}
		return false;
	}
	
	public String toJSON(){
		return "\"" + this.name + "\"";
	}
}
