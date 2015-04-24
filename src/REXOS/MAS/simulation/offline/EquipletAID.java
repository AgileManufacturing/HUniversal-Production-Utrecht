package MAS.simulation.offline;

import jade.core.AID;

import org.apache.commons.lang.builder.HashCodeBuilder;

public class EquipletAID extends AID {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private String equiplet;

	public EquipletAID(String equiplet) {
		this.equiplet = equiplet;
	}
	
	@Override
	public String getLocalName() {
		return equiplet;
	}

	@Override
	public int hashCode() {
		// two randomly chosen prime numbers
		return new HashCodeBuilder(13, 17).append(equiplet).toHashCode();
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null) {
			return false;
		}
		if (obj == this) {
			return true;
		}
		if (obj.getClass() != getClass()) {
			return false;
		}
		EquipletAID aid = (EquipletAID) obj;
		return aid.getEquiplet() == aid.getEquiplet();
	}

	public String getEquiplet() {
		return equiplet;
	}

	@Override
	public String toString() {
		return equiplet;
	}

}
