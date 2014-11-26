package MAS.util;

public class Lock {
	private boolean isLocked = false;

	public synchronized void lock() throws InterruptedException {
		while (isLocked) {
			wait();
		}
		isLocked = true;
	}

	public synchronized void unlock() {
		//System.out.println("Simulation: lock.unlock();");
		isLocked = false;
		notify();
	}

	public synchronized void await() throws InterruptedException {
		while (isLocked) {
			wait();
		}
	}
}