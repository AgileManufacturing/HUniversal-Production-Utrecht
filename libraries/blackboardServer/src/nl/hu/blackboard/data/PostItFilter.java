package nl.hu.blackboard.data;

import org.apache.commons.lang.Validate;
import org.openbbs.blackboard.EntryFilter;
import nl.hu.blackboard.data.PostItProtos.PostItBox;
import nl.hu.blackboard.data.PostItProtos.PostIt;
// TODO: Auto-generated Javadoc
//takes postIts from the blackboard. Only from the boxOwner and if the postit hasnt been processed already by the boxOwner.
/**
 * The Class PostItFilter.
 */
public class PostItFilter implements EntryFilter{

	/** The filter object box. */
	private PostItBox filterObjectBox = null;

	/**
	 * Instantiates a new post it filter.
	 *
	 * @param filterObjectBox the filter object box
	 */
	public PostItFilter(PostItBox filterObjectBox) {
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
		if(pi.getOwner().equals(filterObjectBox.getReadOwner()) && !pi.getIsProcessed())
			return true;
		else 
			return false;
	}
	

}
