/**
 * @file Logging.java
 * @brief TODO
 * @date Created: 24 jun. 2013
 * 
 * @author Theodoor
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2013, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package test.java;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import jade.core.AID;
import jade.core.Agent;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;
import rexos.mas.hardware_agent.behaviours.CheckForModules;
import rexos.mas.hardware_agent.behaviours.EvaluateDuration;
import rexos.mas.hardware_agent.behaviours.FillPlaceholders;
import rexos.mas.hardware_agent.behaviours.ServiceAgentDied;
import rexos.mas.hardware_agent.*;
import rexos.libraries.blackboard_client.BasicOperationSubscription;
import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.BlackboardSubscriber;
import rexos.libraries.blackboard_client.FieldUpdateSubscription;
import rexos.libraries.blackboard_client.FieldUpdateSubscription.MongoUpdateLogOperation;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.MongoOperation;
import rexos.libraries.blackboard_client.OplogEntry;
import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.log.Logger;
import rexos.mas.data.DbData;
import rexos.mas.data.StepStatusCode;
import rexos.mas.service_agent.ServiceStep;

import org.bson.types.ObjectId;
import org.junit.Test;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import rexos.mas.data.Part;
import rexos.mas.data.Position;
import rexos.mas.data.Product;
import rexos.mas.data.ProductLog;
import rexos.mas.data.ProductStep;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.ScheduleData;
import rexos.mas.hardware_agent.HardwareAgent;;



/**
 * @author Theodoor
 * 
 */
public class Logging{
	/**
	 * Test method for
	 * {@link rexos.mas.data.ProductLog#ProductLog(boolean, boolean, rexos.mas.data.sqldatabase.sqliteDatabase)}
	 * .
	 */
	@Test
	public void testProductLog(){
		fail("Not yet implemented");
	}

	/**
	 * Test method for {@link rexos.mas.data.ProductLog#add(java.util.List)}.
	 */
	@Test
	public void testAddListOfLogMessage(){
		fail("Not yet implemented");
	}

	/**
	 * Test method for {@link rexos.mas.data.ProductLog#pushLocalToRemote()}.
	 */
	@Test
	public void testPushLocalToRemote(){
		fail("Not yet implemented");
	}

	/**
	 * Test method for
	 * {@link rexos.mas.data.ProductLog#add(jade.core.AID, com.mongodb.BasicDBObject)}
	 * .
	 * 
	 * @throws Exception
	 */
	@SuppressWarnings("static-method")
	@Test
	public void testAddAIDBasicDBObject() throws Exception{
		BasicDBObject parameters = new BasicDBObject();
		parameters.append("startPosition",
				new Position(1.0, 1.0).toBasicDBObject());
		parameters.append("endPosition",
				new Position(200.0, 200.0).toBasicDBObject());
		// Next we want to have some production steps
		ProductionStep stp1 = new ProductionStep(1, 3, parameters);
		ArrayList<ProductionStep> stepList = new ArrayList<>();
		stepList.add(stp1);
		Production production = new Production(stepList);
		String exampleOfAnAidString = "MainAgent@145.89.100.2521099JADE  addresses (sequence httpLaptop_3_Win_87778acc ))";
		String TestAidString = exampleOfAnAidString
				+ ((Double) Math.random()).toString();
		Product p = new Product(production, new AID(TestAidString, true));
		BasicDBObject update, statusData;
					
			BasicDBObject log = new BasicDBObject();
			BasicDBObject dbEquipletSteps;
			dbEquipletSteps = new BasicDBObject();
			BasicDBObject lookUpParameters = new BasicDBObject();
			
			InstructionData instructionData = new InstructionData();

			dbEquipletSteps.append("test", new EquipletStep(null, 10, instructionData , StepStatusCode.EVALUATING, new BasicDBObject(),
					new TimeData(4)));
			
			EquipletStep[] equipletSteps = new EquipletStep[dbEquipletSteps.size()];
			for(int i = 0; i < dbEquipletSteps.size(); i++) {
				equipletSteps[i] = new EquipletStep((BasicDBObject) dbEquipletSteps.get(i));
			}
			equipletSteps = EquipletStep.sort(equipletSteps);

			// append all equipletsteps to the log
			for(int i = 0; i < equipletSteps.length; i++) {
				log.append("step" + i, equipletSteps[i].toBasicDBObject());
			}
	
			
			statusData = new BasicDBObject("source", "service agent").append(
					"reason", "died").append("log", log);
			
		
			
			
			update = new BasicDBObject("status", StepStatusCode.FAILED.name());
			update.append("statusData", statusData);
			BasicDBObject serviceStepStatusData = new BasicDBObject();
			
			
			
			update.append("statusData", serviceStepStatusData);
			p.getLog().add(new AID(), statusData);
	}
}
