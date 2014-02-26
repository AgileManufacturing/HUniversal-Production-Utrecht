package HAL;

import com.mongodb.util.JSON;

public class ProductStep {
	private Service service;
	private JSON criteria;
	private int id;
	
	public Service getService(){
		return this.service;
	}
	public JSON getCriteria(){
		return this.criteria;
	}
	public int getId(){
		return this.id;
	}
}
