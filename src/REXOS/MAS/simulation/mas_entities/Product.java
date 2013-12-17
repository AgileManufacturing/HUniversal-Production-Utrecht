package simulation.mas_entities;

public class Product {
	
	private ProductStep[] productSteps;
	private long deadline;
	
	public Product(ProductStep[] productSteps, long deadline){
		this.productSteps = productSteps;
		this.deadline = deadline;
	}
	
	
	private void plan(){}
	
	private void schedule(){} //with matrix
	
	private void reschedule(){} //
}
