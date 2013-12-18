package simulation;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

public class Simulation implements Runnable{
	private boolean isRunning = false;
	private double interval = 0; // seconds
	private double turnTime = 0.5; // seconds
	
	private long turn = 0;
	private Date startDate = new Date();
	private ArrayList<Updateable> updateables = new ArrayList<Updateable>();
	private Thread thread;
	
	public void pauseSimulation(){
		isRunning = false;
	}
	public synchronized void resumeSimulation(){
		isRunning = true;
		notify();
	}
	
	public Simulation() {
		thread = new Thread(this);
		thread.start();
	}
	
	public void addUpdateable(Updateable updateable) {
		updateables.add(updateable);
	}
	
	public void run(){
		while(true) {
			synchronized (this) {
				long cycleStartTime = System.currentTimeMillis();
				if(isRunning == false) {
					try {
						System.out.println("Simulation halted on turn " + turn);
						wait();
					} catch (InterruptedException e) {
						// ignore, this is expected
					}
				}
				
				System.out.println("--- Simulation: turn " + turn);
				
				// update the time
				Calendar calender = Calendar.getInstance();
				calender.setTime(startDate);
				calender.add(Calendar.MILLISECOND, (int)(turnTime * turn));
				Date time = calender.getTime();
				
				// update simulation
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
			// TODO remove this
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		// exit simulation thread
	}
}
