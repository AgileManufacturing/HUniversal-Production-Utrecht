package simulation.data;

public class BatchDescription {
	private int id;
	private ProductDescription product;
	private int numberOfRuns;
	
	private static int idCounter = 0;
	
	public static BatchDescription DummyBatch = new BatchDescription(ProductDescription.DummyProduct, 100);
	
	public BatchDescription(ProductDescription prod, int numRuns) {
		id = idCounter++;
		product = prod;
		numberOfRuns = numRuns;
	}
	
	public int getId() {
		return id;
	}
	
	public ProductDescription getProduct() {
		return product;
	}
	
	public int getNumberOfRuns() {
		return numberOfRuns;
	}
	
	public void setId(int id) {
		this.id = id;
	}
	
	public void setProductId(ProductDescription prod) {
		product = prod;
	}
	
	public void setNumberOfRuns(int numRuns) {
		numberOfRuns = numRuns;
	}
	
	public String toString() {
		return product.getName() + " x " + numberOfRuns;
	}
	
	public String toCsvString() {
		return id + "," + product.getId() + "," + numberOfRuns + "\r\n";
	}
}
