package simulation.data;

public class EquipletDescription {
	private String name;
	private Capability[] capabilities;
	private int reservedFor;
	
	public static EquipletDescription DummyEquiplet = new EquipletDescription("Dummy EQ", new Capability[] { Capability.DummyCapability });
	
	public EquipletDescription(String nm, Capability[] caps) {
		this(nm, caps, -1);
	}
	
	public EquipletDescription(String nm, Capability[] caps, int reserved) {
		name = nm;
		capabilities = caps;
		reservedFor = reserved;
	}
	
	public String getName() {
		return name;
	}
	
	public Capability[] getCapabilities() {
		return capabilities;
	}
	
	public int getReservationId() {
		return reservedFor;
	}
	
	public void setName(String nm) {
		name = nm;
	}
	
	public void setCapabilities(Capability[] caps) {
		capabilities = caps;
	}
	
	public void setReservation(int batchId) {
		reservedFor = batchId;
	}
	
	public String toString() {
		String s = "";
		s = name + " { ";
		for(int i = 0; i < capabilities.length; i++) {
			s += capabilities[i].getName();
			
			if(i < capabilities.length - 1) {
				 s += ", ";
			}
		}
		if(reservedFor != -1) {
			s += " }{ " + reservedFor + " }";
		} else {
			s += " }";
		}
		return s;
	}
	
	public String toJsonString() {
		String s = "\t\t\t{\n"
				+ "\t\t\t\t\"name\" : \"" + name + "\",\n"
				+ "\t\t\t\t\"capabilities\" : [";
		
		for(int i = 0; i < capabilities.length; i++) {
			s += capabilities[i].getId();
			if(i < capabilities.length - 1) {
				s += ", ";
			}
		}
		
		s += "],\n"
			+ "\t\t\t\t\"reservedFor\" : " + reservedFor + "\n"
			+ "\t\t\t}";
		
		return s;
	}
}
