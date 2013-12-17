package simulation.mas_entities;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.StringTokenizer;

public class CSVReader {
	
	//example : "1,1,3,4,5,2"
	
	public static int[] parseCapabilitiesCSV(String equipletCapabilitiesFilePathName){
		
		File csvFile = new File(equipletCapabilitiesFilePathName);
		String capabilitiesCSV = "";
		try {
			BufferedReader bReader = new BufferedReader(new FileReader(csvFile));
			
			capabilitiesCSV = bReader.readLine();
		} catch (FileNotFoundException e1) {
			System.err.println("Could not find file: " + equipletCapabilitiesFilePathName);
			e1.printStackTrace();
		} catch (IOException e) {
			System.err.println("Error when reading from file: " + equipletCapabilitiesFilePathName);
			e.printStackTrace();
		}
		
		
		StringTokenizer stringTokenizer = new StringTokenizer(capabilitiesCSV, ",");
		int[] capabilities = new int[stringTokenizer.countTokens()];
		int iCapabilities = 0;
		while (stringTokenizer.hasMoreTokens()){
			try{
			capabilities[iCapabilities] = Integer.parseInt(stringTokenizer.nextToken());
			}
			catch(NumberFormatException e){
				System.err.println("Could not parse capabilties");
				e.printStackTrace();
				return null;
			}
		}
		return capabilities;
	}
}
