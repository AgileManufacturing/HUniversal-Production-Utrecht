package HAL;

public class ModuleIdentifier {
	private String manufacturer;
	private String typeNumber;
	private String serialNumber;
	
	public void setManufacturer(String manufacturer){
		this.manufacturer = manufacturer;
	}
	public void setTypeNumber(String typeNumber){
		this.typeNumber = typeNumber;
	}
	public void setSerialNumber(String serialNumber){
		this.serialNumber = serialNumber;
	}

	public String getManufacturer(){
		return this.manufacturer;
	}
	public String getTypeNumber(){
		return this.typeNumber;
	}
	public String getSerialNumber(){
		return this.serialNumber;
	}
}