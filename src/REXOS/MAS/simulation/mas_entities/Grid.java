package simulation.mas_entities;

import java.io.File;
import java.io.Reader;
import java.util.Date;

import agents.data_classes.Matrix;
import simulation.CSVReader;
import simulation.Updateable;
import simulation.data.Capability;


public class Grid implements Updateable{
	private static final double defaultDistance = 1.0;
	
	private Equiplet[][] equiplets;
	
	private Matrix distanceMatrix;
	
	public Grid(Equiplet[][] equiplets) {
		this.equiplets = equiplets;
		distanceMatrix = new Matrix(equiplets.length * equiplets[0].length, equiplets.length * equiplets[0].length);
	}
	public Grid(String equipletLayoutCsvFilePath) {
		String[][] fields = CSVReader.parseCsvFile(equipletLayoutCsvFilePath);
		
		distanceMatrix = new Matrix(fields.length * fields[0].length, fields.length * fields[0].length);
		initializeDistanceMatrix();
		
		equiplets = new Equiplet[fields.length][fields[0].length];
		try {
			for(int i = 0; i < fields.length; i++) {
				for(int j = 0; j < fields[i].length; j++) {
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
	private void initializeDistanceMatrix() {
		for(int i = 0; i < distanceMatrix.getNumberOfRows(); i++) {
			int sourceEquipletY = i / equiplets.length; 
			int sourceEquipletX = i % equiplets.length;
			for(int j = 0; j < distanceMatrix.getNumberOfColumns(); j++) {
				int targetEquipletY = j / equiplets.length; 
				int targetEquipletX = j % equiplets.length;
				
				double distance = 
						Math.abs(targetEquipletY - sourceEquipletY) * defaultDistance + 
						Math.abs(targetEquipletX - sourceEquipletX) * defaultDistance;
				distanceMatrix.set(i, j, distance);
			}
		}
	}
	
	public Matrix getDistanceMatrix(){
		return distanceMatrix;
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
