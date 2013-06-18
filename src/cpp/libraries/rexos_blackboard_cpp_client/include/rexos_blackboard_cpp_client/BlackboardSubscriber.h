/**
 * @file BlackboardSubscriber.h
 * @brief Interface providing callback functions for the blackboard client.
 * @date Created: 2012-11-19
 *
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#ifndef BLACKBOARD_SUBSCRIBER_H_
#define BLACKBOARD_SUBSCRIBER_H_

namespace Blackboard
{
class BlackboardSubscription;
class OplogEntry;

/**
 * Interface providing callback functions for the blackboard client.
 **/
class BlackboardSubscriber {
public:
	/**
	 * This callback is invoked whenever an oplog entry is parsed matching a subscription.
	 * @param subscription Reference to the BlackboardSubscription for which this callback was invoked.
	 * @param oplogEntry Reference to the OplogEntry containing all information about the event.
	 */
	virtual void onMessage(BlackboardSubscription & subscription, const OplogEntry & oplogEntry) = 0;

	/**
	 * Virtual destructor to make sure child classes will be able to clean up.
	 */
	virtual ~BlackboardSubscriber(){}
};

}
#endif
