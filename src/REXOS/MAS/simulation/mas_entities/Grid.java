package simulation.mas_entities;

import java.io.File;
import java.io.Reader;
import java.util.Date;

import simulation.CSVReader;
import simulation.Updateable;
import simulation.data.Capability;


public class Grid implements Updateable{
	private Equiplet[][] equiplets;
	
	public Grid(Equiplet[][] equiplets) {
		this.equiplets = equiplets;
	}
	public Grid(String equipletLayoutCsvFilePath) {
		String[][] fields = CSVReader.parseCsvFile(equipletLayoutCsvFilePath);
		
		equiplets = new Equiplet[fields.length][fields[0].length];
		try {
			for(int i = 0; i < fields.length; i++) {
				for(int j = 0; j < fields[0].length; j++) {
					int capabilityId = Integer.parseInt(fields[i][j].trim());
					Capability cap = Capability.getAvailableCapabilitiesById(capabilityId);
					// TODO: Allow for more than 1 capability
					equiplets[i][j] = new Equiplet(new Capability[] {cap});
				}
			}
		} catch (NumberFormatException ex) {
			System.err.println("equipletLayoutCsv has an entry which could not be converted to int");
			throw ex;
		}
	}
	@Override
	public void update(Date time) {
		for(int i = 0; i < equiplets.length; i++) {
			for(int j = 0; j < equiplets[i].length; j++) {
				equiplets[i][j].update(time);
			}
		}
		
	}
}
