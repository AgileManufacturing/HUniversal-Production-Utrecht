package MAS.simulation.offline;

class ProductStep {
	private String service;

	public ProductStep(String service) {
		this.service = service;
	}

	public String getService() {
		return service;
	}

	@Override
	public String toString() {
		return String.format("Step:[service=%s]", service);
	}
}
