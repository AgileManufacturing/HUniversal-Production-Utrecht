package simulation.data;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;


public class Capability {
	private int id;
	private String name;
	
	private static Capability[] availableCapabilities;
	
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
	
	public static void parseCapabilitiesCSV(String capabilitiesFilePath){
		
		File csvFile = new File(capabilitiesFilePath);
		String capabilitiesCSV = "";
		try {
			BufferedReader bReader = new BufferedReader(new FileReader(csvFile));
			
			capabilitiesCSV = bReader.readLine();
			bReader.close();
		} catch (FileNotFoundException e1) {
			System.err.println("Could not find file: " + capabilitiesFilePath);
			e1.printStackTrace();
		} catch (IOException e) {
			System.err.println("Error when reading from file: " + capabilitiesFilePath);
			e.printStackTrace();
		}
		
		StringTokenizer commaTokenizer = new StringTokenizer(capabilitiesCSV, ",");
		Capability[] capabilities = new Capability[commaTokenizer.countTokens()];
		int iCapabilities = 0;
		while (commaTokenizer.hasMoreTokens()){
			try{
				StringTokenizer semicolonTokenizer = new StringTokenizer(commaTokenizer.nextToken());
				int capabilityId = Integer.parseInt(semicolonTokenizer.nextToken());
				String capabiityName = semicolonTokenizer.nextToken();
				availableCapabilities[iCapabilities] = new Capability(capabilityId, capabiityName);
				
				iCapabilities ++;
			}
			catch(NumberFormatException e){
				System.err.println("Could not parse capabilties");
				e.printStackTrace();
			}
			catch(NoSuchElementException e1){
				System.err.println("wrong amount of elements when parsing capabilities");
				e1.printStackTrace();
			}
		}
		
		availableCapabilities = capabilities;
	}
	
	public static Capability getAvailableCapabilitiesById(int id){
		return availableCapabilities[id];
	}
}
