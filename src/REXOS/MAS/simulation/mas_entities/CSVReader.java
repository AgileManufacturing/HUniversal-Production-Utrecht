package simulation.mas_entities;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;

import simulation.data.Capability;

public class CSVReader {
	
	//example : "1;name,1;name,3;appel,4;peer,5;hurp,2;durp"
	
	
	private static Capability[] availableCapabilities;
		
	public static void parseCapabilitiesCSV(String equipletCapabilitiesFilePathName){
		
		File csvFile = new File(equipletCapabilitiesFilePathName);
		String capabilitiesCSV = "";
		try {
			BufferedReader bReader = new BufferedReader(new FileReader(csvFile));
			
			capabilitiesCSV = bReader.readLine();
			bReader.close();
		} catch (FileNotFoundException e1) {
			System.err.println("Could not find file: " + equipletCapabilitiesFilePathName);
			e1.printStackTrace();
		} catch (IOException e) {
			System.err.println("Error when reading from file: " + equipletCapabilitiesFilePathName);
			e.printStackTrace();
		}
		
		StringTokenizer commaTokenizer = new StringTokenizer(capabilitiesCSV, ",");
		availableCapabilities = new Capability[commaTokenizer.countTokens()];
		int iCapabilities = 0;
		while (commaTokenizer.hasMoreTokens()){
			try{
				StringTokenizer semicolonTokenizer = new StringTokenizer(commaTokenizer.nextToken(), ";");
				
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
		printAvailableCapabilities();
	}
	
	private static void printAvailableCapabilities(){
		for ( Capability c1 : availableCapabilities){
			System.out.println("id: " + c1.getId() + ", name: " + c1.getName());
		}
	}
	
	
}
