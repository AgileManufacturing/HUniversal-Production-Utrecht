package simulation;

import java.text.ParseException;

import simulation.data.Capability;
import simulation.mas_entities.Batch;
import simulation.mas_entities.Grid;
import simulation.mas_entities.Product;

public class BatchSpawner extends ProductSpawner{
	private static final String pathToBatchCsvFile = "/home/t/sim/BatchA.csv";
	
	protected int batchGroup;

	public BatchSpawner(Simulation simulation, Grid grid) throws ParseException {
		super(simulation, grid);
		
		
		String[][] fields = CSVReader.parseCsvFile(pathToBatchCsvFile);
		
		productName = fields[0][0];
		productDeadline = Duration.parseDurationString(fields[1][0]);
		long start = Duration.parseDurationString(fields[2][0]);
		nextSpawnTime = simulation.getCurrentSimulationTime() + start;
		
		amountPerSpawn = Integer.parseInt(fields[3][0]);
		spawnInterval = Integer.parseInt(fields[4][0]);
		batchGroup = Integer.parseInt(fields[5][0]);
		
		productStepCapabilities = new Capability[fields.length - 6];
		for(int i = 6; i < fields.length; i++) {
			productStepCapabilities[i - 6] = Capability.getCapabilityByName(fields[i][0]);
		}		
	}
	
	@Override
	public void update(long time) {
		if(time >= nextSpawnTime) {
			Product[] products = spawnBatch();
			for(int i = 0; i < products.length; i++) {
				simulation.addUpdateable(products[i]);
			}
			
			nextSpawnTime += spawnInterval * 1000;
			System.out.println("BatchSpawner: Spawned " + amountPerSpawn + " " + productName + "s, next spawn in " + spawnInterval + " (" + nextSpawnTime + ")");
		}

	}
	private Product[] spawnBatch() {
		Batch batch = new Batch(batchGroup);
		Product[] products = new Product[amountPerSpawn];
		
		for(int i = 0; i < products.length; i++) {
			products[i] = new Product(productName, simulation, grid, productStepCapabilities, simulation.getCurrentSimulationTime() + productDeadline, batch);
			batch.addProduct(products[i]);
		}
		return products;
	}
}
