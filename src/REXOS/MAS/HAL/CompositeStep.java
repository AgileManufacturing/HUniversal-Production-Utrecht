package HAL;

import com.mongodb.util.JSON;

public class CompositeStep {
	private ProductStep productStep;
	private JSON command;
	
	public ProductStep getProductStep(){
		return this.productStep;
	}
	public JSON getCommand(){
		return command;
	}
}
