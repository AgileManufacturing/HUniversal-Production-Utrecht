package simulation;

import java.util.ArrayList;
import java.util.Date;

public class Simulation implements Runnable{
	private boolean isRunning = false;
	private double interval = 0; // seconds
	
	private long turn = 0;
	private Date startDate = new Date();
	private ArrayList<Updateable> updateables;
	
	public void pauseSimulation(){
		isRunning = false;
	}
	public void resumeSimulation(){
		isRunning = true;
		notifyAll();
	}
	
	public void run(){
		while(true) {
			long cycleStartTime = System.currentTimeMillis();
			if(isRunning == false) {
				try {
					wait();
				} catch (InterruptedException e) {
					// ignore, this is expected
				}
			}
			
			// update simulation
			Date time = new Date();
			
			for (Updateable updateable : updateables) {
				updateable.update(time);
			}
			
			turn++;
			// sleep if dealing with 'realtime' simulation
			long cycleEndTime = System.currentTimeMillis();
			long cycleDuration = cycleEndTime - cycleStartTime;
			if(cycleDuration < (interval * 1000)) {
				try {
					Thread.sleep((long) ((interval * 1000) - cycleDuration));
				} catch (InterruptedException e) {
					// ignore
				}
			} else {
				// no need to sleep at all
			}
		}
		// exit simulation thread
	}
}
