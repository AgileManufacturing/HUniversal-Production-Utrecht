package simulation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;

import simulation.data.Capability;

public class CSVReader {
	
	//example : "1;name,1;name,3;appel,4;peer,5;hurp,2;durp"
	
	
	private static Capability[] availableCapabilities;
		
	public static String[][] parseCsvFile(String csvFilePath) {
		System.out.println("reading " + csvFilePath);
		
		
		
		File csvFile = new File(csvFilePath);
		String fileString = "";
		try {
			byte[] fileContent = Files.readAllBytes(Paths.get(csvFilePath));
			fileString = Charset.defaultCharset().decode(ByteBuffer.wrap(fileContent)).toString();
		} catch (FileNotFoundException ex) {
			System.err.println("Could not find file: " + csvFilePath);
			ex.printStackTrace();
		} catch (IOException ex) {
			System.err.println("Error when reading from file: " + csvFilePath);
			ex.printStackTrace();
		}
		return parseCsvString(fileString);
	}
	public static String[][] parseCsvString(String csvString) {
		// split on lines (this should work for windows too)
		String lines[] = csvString.split("\\r?\\n");
		int rowCount = lines.length;
		if(rowCount == 0) {
			// if input is empty, so is output
			return new String[0][0];
		}
		// assuming every row has the same number of cols 
		int colCount = lines[0].split(";").length;
		
		String[][] output = new String[rowCount][colCount];
		for(int i = 0; i < lines.length; i++) {
			String[] fields = lines[i].split(";");
			output[i] = fields;
		}
		return output;
	}
}
