package nl.hu.blackboard.data;

import java.io.Serializable;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.EntryFilter;
import nl.hu.blackboard.data.PostItProtos.PostIt;
// TODO: Auto-generated Javadoc

/**
 * The Class AvailabilityFilter.
 */
public class AvailabilityFilter implements EntryFilter, Serializable
{
	
	/** The owner. */
	private String theOwner;
	
	/** The deadline. */
	private long deadline;

	/**
	 * Instantiates a new availability filter.
	 *
	 * @param theOwner the the owner
	 * @param deadline the deadline
	 */
	public AvailabilityFilter(String theOwner, long deadline) {
	      Validate.notNull(theOwner);
	      this.theOwner = theOwner;
	      this.deadline = deadline;
	}

	/* (non-Javadoc)
	 * @see org.openbbs.blackboard.EntryFilter#selects(java.lang.Object)
	 */
	@Override
	public boolean selects(Object entry) {
		PostIt pi = (PostIt)entry;
		if(pi.getOwner().equals(theOwner) && pi.getTimestep() < deadline)
			return true;
		else
			return false;
	}
	
	
	
	
	
}
