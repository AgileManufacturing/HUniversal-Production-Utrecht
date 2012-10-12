package nl.hu.blackboard.data;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.EntryFilter;
import nl.hu.blackboard.data.PostItProtos.PostItBox;
import nl.hu.blackboard.data.PostItProtos.PostIt;

// TODO: Auto-generated Javadoc
/**
 * The Class CleanBlackboardFilter.
 */
public class CleanBlackboardFilter implements EntryFilter{

	/** The filter object box. */
	private PostItBox filterObjectBox = null;

	/**
	 * Instantiates a new clean blackboard filter.
	 *
	 * @param filterObjectBox the filter object box
	 */
	public CleanBlackboardFilter(PostItBox filterObjectBox) {
	      Validate.notNull(filterObjectBox);
	      this.filterObjectBox = filterObjectBox;
	}
	
	/* (non-Javadoc)
	 * @see org.openbbs.blackboard.EntryFilter#selects(java.lang.Object)
	 */
	@Override
	public boolean selects(Object entry) 
	{		 
		PostIt pi = (PostIt)entry;	
		
		if(pi == null)
			System.out.println("null");
		else
			System.out.println(filterObjectBox.getReadOwner() + ":"  +  pi.getOwner() + " :load " +  pi.getPayload() +   " :time"+ pi.getTimestep() + ":curr = " + filterObjectBox.getCurrentTimeStep());
		
		if(pi.getOwner().equals(filterObjectBox.getReadOwner()) && (filterObjectBox.getCurrentTimeStep() > pi.getTimestep() || (pi.getTimestep() == 0 && filterObjectBox.getCurrentTimeStep() == 0  )) && pi.getIsProcessed())
			return true;
		else		
			return false;
	}

}
