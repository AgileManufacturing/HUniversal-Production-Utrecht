package simulation;

import java.util.Date;

public class ProductSpawner implements Updateable {
	Date startDate;
	Date nextSpawn;
	double interval;
	
	@Override
	public void update(Date time) {
		if(time.after(startDate)) {
			
		}

	}

}
