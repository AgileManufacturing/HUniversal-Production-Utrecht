package HAL;

import com.mongodb.util.JSON;

public class HardwareStep {
	private CompositeStep compositeStep;
	private JSON command;
	
	public CompositeStep getCompositeStep(){
		return this.compositeStep;
	}
	public JSON getCommand(){
		return this.command;
	}
}
