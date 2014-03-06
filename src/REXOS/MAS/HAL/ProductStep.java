package HAL;

import java.io.Serializable;

import com.google.gson.Gson;

public class ProductStep implements Serializable {
	private static final long serialVersionUID = 6788269249283740246L;
	private Service service;
	private Gson criteria;
	private int id;
	
	public Service getService(){
		return this.service;
	}
	public Gson getCriteria(){
		return this.criteria;
	}
	public int getId(){
		return this.id;
	}
}
