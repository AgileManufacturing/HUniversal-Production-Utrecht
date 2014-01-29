package simulation.data;

import java.util.ArrayList;

import simulation.Duration;

public class ProductDescription {
	private String name;
	private int id;
	private long deadline;
	private int count;
	private int intervalInSeconds;
	private ArrayList<ProductStep> productSteps;
	
	private static ArrayList<ProductStep> dummySteps = new ArrayList<ProductStep>();
	public static ProductDescription DummyProduct = new ProductDescription("Dummy Product", Duration.parseDurationString("0:0:1"), 1, 1, dummySteps);
	
	private static int idCounter = 0;
	
	static {
		dummySteps.add(ProductStep.DummyProductStep);
	}
	
	public ProductDescription(long deadline, int count, int interval, ArrayList<ProductStep> steps) {
		this("", deadline, count, interval, steps);
		this.setName("Product-" + hashCode());
	}
	
	public ProductDescription(String name, long deadline, int count, int interval, ArrayList<ProductStep> steps) {
		this.name = name;
		this.deadline = deadline;
		this.count = count;
		intervalInSeconds = interval;
		productSteps = steps;
		id = idCounter++;
	}
	
	public String getName() {
		return name;
	}
	
	public int getId() {
		return id;
	}
	
	public long getDeadline() {
		return deadline;
	}
	
	public int getCount() {
		return count;
	}
	
	public int getIntervalInSeconds() {
		return intervalInSeconds;
	}
	
	public ArrayList<ProductStep> getProductSteps() {
		return productSteps;
	}
	
	public void setName(String name) {
		this.name = name;
	}
	
	public void setDeadline(long dur) {
		deadline = dur;
	}
	
	public void setCount(int count) {
		this.count = count;
	}
	
	public void setIntervalInSeconds(int seconds) {
		intervalInSeconds = seconds;
	}
	
	public void setProductSteps(ArrayList<ProductStep> steps) {
		productSteps = steps;
	}
	
	public String toString() {
		return name + " { " + deadline + ", " + count + ", " + intervalInSeconds + " }";
	}
	
	public String toCsvString() {
		String prodSteps = "";
		for(int i = 0; i < productSteps.size(); i++) {
			prodSteps += "," + productSteps.get(i).getCapability().getId(); 
		}
		return id + "," + deadline + "," + count + "," + intervalInSeconds + prodSteps;
	}
}
