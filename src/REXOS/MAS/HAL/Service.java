package HAL;

import java.io.Serializable;

public class Service implements Serializable {
	private static final long serialVersionUID = 1067152669716744380L;
	private String name;
	
	public String getName(){
		return this.name;
	}
}
