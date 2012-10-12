//******************************************************************************
//
//                 Blackboard Server
//
//******************************************************************************
// Project:        Blackboard Server
// File:           PostItFilter.java	  
// Description:    ?
// Author:         Pascal Muller
// Notes:          
//
// License:        newBSD
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

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
