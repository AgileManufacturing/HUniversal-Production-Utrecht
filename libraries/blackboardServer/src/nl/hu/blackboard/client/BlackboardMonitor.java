package nl.hu.blackboard.client;


// TODO: Auto-generated Javadoc
/**
 * The Class BlackboardMonitor.
 */
public class BlackboardMonitor 
{

	/** The locked. */
	private boolean locked = false;
	
	
	/**
	 * Gets the key.
	 *
	 * @return the key
	 */
	public synchronized boolean getKey()
	{
		if(!locked)
		{
			locked = true;						
			return true;
		}
		else
		{		
			return false;	
		}
	}
	
	/**
	 * Give key back.
	 */
	public void giveKeyBack()
	{			
		locked = false;		
	}
}
