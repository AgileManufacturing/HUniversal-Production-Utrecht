/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
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


#include <RosBlackboardSendNode/RosBlackboardSendNode.h>


/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
RosBlackboardSendNode::RosBlackboardSendNode(int messages): messageCount(0),messages(messages)
{
	ROS_INFO("SENDING %d",messages);
	blackboardClient = new BlackboardCppClient("localhost", "REXOS", "blackboard", this);
	blackboardClient->subscribe("receive");
} 

/**
 * Destructor for the EquipletNode
 **/
RosBlackboardSendNode::~RosBlackboardSendNode() {
	delete blackboardClient;
}

/**
 * This function is called when a new message on the Blackboard is received,
 * The command, destination and payload are read from the message, and the 
 * service specified in the message is called
 *
 * @param event The blackboard event that occured: added, updated, deleted
 * @param json The message parsed in the json format
 **/
void RosBlackboardSendNode::blackboardReadCallback(std::string json) {
	timespec receiveTime;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &receiveTime);
	blackboardClient->removeOldestMessage();
		
		
	// timeing
	if(receiveTime.tv_nsec - sendTime.tv_nsec >= 0)
	{
		messageCount++;
		times.push_back(receiveTime.tv_nsec - sendTime.tv_nsec);
	}
	if(messageCount < messages)
	{
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &sendTime);
		blackboardClient->insertJson("{ topic:\"send\", data:\"................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................\" }");
	}
	else
	{
		std::ofstream outputFile("data.txt");
		for(std::vector<float>::iterator it = times.begin(); it != times.end(); ++it)
		{
			 outputFile <<  boost::lexical_cast<std::string>( *it);
			 outputFile << "\n";
		}
		std::cout << "done!" << std::endl;	
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ROS_BLACKBOARD_SEND_NODE");
	ros::NodeHandle handle;
	RosBlackboardSendNode RosBlackboardSendNode(atoi(argv[1]));
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &RosBlackboardSendNode.sendTime);
	sleep(1);
	ROS_INFO("SENDING STARTED! OMGS");
	RosBlackboardSendNode.blackboardClient->insertJson("{ topic:\"send\", data:\"................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................\" }");
	mongo::DBClientConnection connection;
	connection.connect("localhost");
	while(ros::ok())
	{
		
		/*mongo::Query where = QUERY("ns" << "REXOS.blackboard");
		//mongo::BSONObj last_element = connection.findOne("local.oplog.rs", where.sort("$natural", -1)); 
		
		//mongo::BSONElement o = last_element["o"];
		//std::cout << o["_id"];
		mongo::BSONObj ret;
		mongo::BSONObj bsonQry = BSON("collStats" << "oplog.rs");
		connection.runCommand( "local", bsonQry, ret);  
		std::cout << "skipping " << ret.getIntField("count") << "entries" << std::endl; 

		//where = QUERY("ns" << "REXOS.blackboard" << "o._id" << mongo::GTE << o["_id"]).sort("$natural");
		//std::cout << where << std::endl;
		mongo::auto_ptr<mongo::DBClientCursor> tailedCursor = connection.query("local.oplog.rs", where, 0,
		ret.getIntField("count"), 0, mongo::QueryOption_CursorTailable | mongo::QueryOption_NoCursorTimeout |mongo::QueryOption_SlaveOk | mongo::QueryOption_AwaitData, 0

);		
		std::cout << "cursor is tailable: " << tailedCursor->tailable() << std::endl;
		int i =0;
		while(true){
		std::cout << tailedCursor->isDead() << std::endl;
		sleep(1);
		
			while(tailedCursor->more())
			{
			
				i++;
tailedCursor->next(); 
				//std::cout << tailedCursor->next() << std::endl;
			}	
		std::cout << "found  " << i << " records";
		}
		*/
		
		
	}
	
		
	
	

	return 0;
}
