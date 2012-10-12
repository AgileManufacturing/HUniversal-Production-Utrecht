package nl.hu.blackboard.data;

import java.io.Serializable;
import java.util.ArrayList;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.EntryFilter;
import nl.hu.blackboard.data.PostItProtos.PostIt;


// TODO: Auto-generated Javadoc
/**
 * The Class InitiateRescheduleFilter.
 */
public class InitiateRescheduleFilter implements Serializable, EntryFilter{

	/** The products to neutralize. */
	private ArrayList<String> productsToNeutralize;
	

	/**
	 * Instantiates a new initiate reschedule filter.
	 *
	 * @param products the products
	 */
	public InitiateRescheduleFilter(ArrayList<String> products) {
	      Validate.notNull(products);
	      productsToNeutralize = products;
	}

	/* (non-Javadoc)
	 * @see org.openbbs.blackboard.EntryFilter#selects(java.lang.Object)
	 */
	@Override
	public boolean selects(Object entry) 
	{	
		PostIt pi = (PostIt)entry;
		if(productsToNeutralize.contains(pi.getProductRef()))
		{
			//this postit will be thown away next time
			return true;
		}	
		return false;		
	}
}
