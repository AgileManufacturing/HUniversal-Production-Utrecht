package simulation.data;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;

import simulation.CSVReader;


public class Capability {
	private int id;
	private String name;
	
	private static HashMap<Integer, Capability> availableCapabilities = new HashMap<Integer, Capability>();
	
	public static void loadCapabilities(String capabilitiesFilePath) {
		String[][] fields = CSVReader.parseCsvFile(capabilitiesFilePath);
		parseCapabilitiesCSV(fields);
	}
	
	public Capability(int id, String name){
		this.id = id;
		this.name = name;
	}

	public int getId() {
		return id;
	}

	public String getName() {
		return name;
	}
	
	public static void parseCapabilitiesCSV(String[][] fields){
		try{
			for(int i = 0; i < fields.length; i++) {
				int capabilityId = Integer.parseInt(fields[i][0]);
				String capabiityName = fields[i][1];
				
				availableCapabilities.put(capabilityId, new Capability(capabilityId, capabiityName));
			}
		} catch(NumberFormatException e){
			System.err.println("Could not parse capabilties");
			e.printStackTrace();
		} catch(NoSuchElementException e1){
			System.err.println("wrong amount of elements when parsing capabilities");
			e1.printStackTrace();
		}
		printAvailableCapabilities();
	}
	
	public static Capability getAvailableCapabilitiesById(int id){
		return availableCapabilities.get(id);
	}
	private static void printAvailableCapabilities(){
		Collection<Capability> values = availableCapabilities.values();
		for ( Capability c1 : values){
			System.out.println("id: " + c1.getId() + ", name: " + c1.getName());
		}
	}
}
