package HAL;

import java.util.ArrayList;

public abstract class Capability {
	private String name;
	public String getName() {
		return name;
	}
	
	abstract public ArrayList<HardwareStep> translateProductStep(ProductStep productStep);
	
}
