package HAL;

import java.io.Serializable;

import com.mongodb.util.JSON;

public class CompositeStep implements Serializable{
	private static final long serialVersionUID = 1206944727256435741L;
	private ProductStep productStep;
	private JSON command;
	
	public ProductStep getProductStep(){
		return this.productStep;
	}
	public JSON getCommand(){
		return command;
	}
}
