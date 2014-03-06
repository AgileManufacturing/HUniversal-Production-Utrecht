package HAL;

import java.io.Serializable;

import com.google.gson.JsonObject;

public class ProductStep implements Serializable {
	private static final long serialVersionUID = 6788269249283740246L;
	private Service service;
	private JsonObject criteria;
	private int id;
	
	public ProductStep(int id, JsonObject criteria, Service service){
		this.id = id;
		this.criteria = criteria;
		this.service = service;
	}
	
	public Service getService(){
		return this.service;
	}
	public JsonObject getCriteria(){
		return this.criteria;
	}
	public int getId(){
		return this.id;
	}
}
