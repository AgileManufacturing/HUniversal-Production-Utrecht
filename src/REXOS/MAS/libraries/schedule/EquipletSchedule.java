/**
 * @file REXOS/MAS/libraries/data_classes/ProductStepSchedule.java
 * @brief Data object for the schedule of a product step primarily in the planningblackboard
 * @date Created: 04 nov 2013
 * 
 * @author Roy Scheefhals
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
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


package libraries.schedule;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

import org.bson.BSONObject;
import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;
import com.sun.org.apache.bcel.internal.generic.NEW;
import com.sun.xml.internal.ws.policy.privateutil.PolicyUtils.Collections;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.schedule.data_classes.*;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

public class EquipletSchedule extends Schedule {
	
	private int freeTimeSlotDurationThreshold = 25;
	
	private BlackboardClient planningBlackboard;
	
	private BlackboardClient FreeTimeSlotBlackboard;
	
	private BlackboardClient RealtimeBlackboard;
	
	private String databaseName;
	
	private TimeSlotSynchronization timeSlotSynchronization;
	
	private EquipletScheduleInformation equipletFreeTimeData;
	
	private ObjectId infiniteTimeSlotObjectId;
	
	public EquipletSchedule(String scheduleHostName, int schedulePort, String databaseName, TimeSlotSynchronization timeSlotSynchronization, boolean clearSchedule) 
			throws UnknownHostException, GeneralMongoException, InvalidDBNamespaceException{
		super(scheduleHostName, schedulePort);
		this.databaseName = databaseName;
		this.timeSlotSynchronization = timeSlotSynchronization;
		
		setup();
		
		if (clearSchedule){
			clearSchedule();
		}
	}
	
	private void setup() throws UnknownHostException, GeneralMongoException, InvalidDBNamespaceException{
		planningBlackboard = new BlackboardClient(scheduleHostName, schedulePort);
		FreeTimeSlotBlackboard = new BlackboardClient(scheduleHostName, schedulePort);
		RealtimeBlackboard = new BlackboardClient(scheduleHostName, schedulePort);
		
		planningBlackboard.setDatabase(databaseName);
		FreeTimeSlotBlackboard.setDatabase(databaseName);
		RealtimeBlackboard.setDatabase(databaseName);
		
		planningBlackboard.setCollection(PLANNING_NAME + "Blackboard");
		FreeTimeSlotBlackboard.setCollection(FREETIMESLOT_NAME + "Blackboard");
		RealtimeBlackboard.setCollection(REALTIMESCHEDULE_NAME + "Blackboard");
	}
	
	private void clearSchedule() throws InvalidDBNamespaceException, GeneralMongoException{
		planningBlackboard.removeDocuments(new BasicDBObject());
		FreeTimeSlotBlackboard.removeDocuments(new BasicDBObject());
		RealtimeBlackboard.removeDocuments(new BasicDBObject());
		
		//insert infinite freetimeslot 
		long currentTimeSlot = timeSlotSynchronization.getCurrentTimeSlot();
		FreeTimeSlot freeTimeSlot = new FreeTimeSlot(currentTimeSlot, null);
		
		infiniteTimeSlotObjectId = FreeTimeSlotBlackboard.insertDocument(freeTimeSlot.toBasicDBObject());
	}

	public EquipletScheduleInformation getFreeTimeSlots(Long duration, Long deadline) throws InvalidDBNamespaceException, GeneralMongoException, ScheduleException {
		
		ArrayList<FreeTimeSlot> freeTimeSlots = new ArrayList<FreeTimeSlot>();
		Long infiniteFreeTimeSlot = null;
		//filter freetimeslots on not shorter than given duration
		if (duration != null){
			
		}
		//filter results on times before the deadline
		//also figure out load of the deadline
		if (deadline != null){
			
		}
		//create search query
		//DBObject filterquery = QueryBuilder.start("startTimeSlot").lessThan(deadline).get();
		BasicDBObject orderBy = new BasicDBObject("startTimeSlot", "1");
		
		BasicDBObject findquery = new BasicDBObject("$orderby", orderBy);
		
		List<DBObject> freeTimeSlotDBObjects = FreeTimeSlotBlackboard.findDocuments(findquery);
		
		for(DBObject freeTimeSlotDBObject : freeTimeSlotDBObjects){
			
			FreeTimeSlot newFreeTimeSlot = new FreeTimeSlot((BasicDBObject) freeTimeSlotDBObject); 
			if ( newFreeTimeSlot.getEndTimeSlot() != null){
				freeTimeSlots.add(newFreeTimeSlot);
			}
			else{
				infiniteFreeTimeSlot = newFreeTimeSlot.getStartTimeSlot();
			}
			
		}
		if (infiniteFreeTimeSlot == null){
			throw new ScheduleException("InfiniteFreeTimeSlot not found, Schedule is full or the FreeTimeSlotBlackboard has failed");
		}
		
		equipletFreeTimeData = new EquipletScheduleInformation(freeTimeSlots, infiniteFreeTimeSlot, 50.0d);
		
		return equipletFreeTimeData; 
	}
	
	public void scheduleProductSteps(UUID lockKey, ArrayList<ProductStepSchedule> scheduleData) throws ScheduleAccessException, 
			ScheduleException, InvalidDBNamespaceException, GeneralMongoException {
		super.checkLock(lockKey);
		
		//validate the productsteps
		if (!validateScheduleData(scheduleData)){
			throw new ScheduleException("Given scheduleData does not fit in the current schedule");
		}
		
		for (ProductStepSchedule productStepSchedule : scheduleData){
			planningBlackboard.insertDocument(productStepSchedule.toBasicDBObject());
		}
		
		recalculateFreeTimeSlots();
	}
	
	private void recalculateFreeTimeSlots() throws InvalidDBNamespaceException, GeneralMongoException{

		long currentTimeSlot = timeSlotSynchronization.getCurrentTimeSlot();
		
		List<DBObject> plannedDBObjects =  planningBlackboard.findDocuments(new BasicDBObject());
		
		ArrayList<FreeTimeSlot> newFreeTimeSlots = new ArrayList<FreeTimeSlot>();
		
		if (plannedDBObjects.size() == 0){
			//there are no slots, add new inf timeslot
			newFreeTimeSlots.add(new FreeTimeSlot(currentTimeSlot, null));
		}else if (plannedDBObjects.size() == 1){
			//only one slot, add new possible freetimeslot and add inf timeslot
			ProductStepSchedule prodStepSchedule =  new ProductStepSchedule((BasicDBObject)plannedDBObjects.get(0));
			if (prodStepSchedule.getStartTime() > currentTimeSlot){
				newFreeTimeSlots.add(new FreeTimeSlot(currentTimeSlot, prodStepSchedule.getStartTime() - currentTimeSlot));
			}
			newFreeTimeSlots.add(new FreeTimeSlot(prodStepSchedule.getStartTime() + prodStepSchedule.getDuration(), null));
		}
		else if (plannedDBObjects.size() > 1){
			ProductStepSchedule curProdStepSchedule;
			ProductStepSchedule prevProdStepSchedule = new ProductStepSchedule((BasicDBObject)plannedDBObjects.get(0));
			for (int iPlannedDBObject = 1; iPlannedDBObject <plannedDBObjects.size(); iPlannedDBObject++){
				curProdStepSchedule = new ProductStepSchedule((BasicDBObject)plannedDBObjects.get(iPlannedDBObject));
				
				// if the 2 prodSteps have a gap between them, and this gap is bigger than the threshhold, add a new freetimeslot
				if (curProdStepSchedule.getStartTime() + 1 - prevProdStepSchedule.getStartTime()+ prevProdStepSchedule.getDuration() > freeTimeSlotDurationThreshold ){
					newFreeTimeSlots.add(new FreeTimeSlot(prevProdStepSchedule.getStartTime()+ prevProdStepSchedule.getDuration() + 1, 
							curProdStepSchedule.getStartTime() - prevProdStepSchedule.getStartTime()+ prevProdStepSchedule.getDuration()));
				}
			}
		}
		
	}

	private boolean validateScheduleData(ArrayList<ProductStepSchedule> scheduleData){
		
		for (ProductStepSchedule productStepSchedule : scheduleData){
			if(!validateProductStepSchedule(productStepSchedule)){
				return false;
			}
		}
		return true;
	}
	
	private boolean validateProductStepSchedule(ProductStepSchedule productStepSchedule){
		
		if (productStepSchedule.getStartTime() >= equipletFreeTimeData.getinfiniteFreeTimeSlot()){
			return true;
		}
		for (FreeTimeSlot freeTimeSlot : equipletFreeTimeData.getFreeTimeSlots()){
			if (productStepSchedule.getStartTime() >= freeTimeSlot.getStartTimeSlot() && 
					productStepSchedule.getDuration() <= freeTimeSlot.getEndTimeSlot() && 
					productStepSchedule.getStartTime() + productStepSchedule.getDuration() <= freeTimeSlot.getStartTimeSlot() + freeTimeSlot.getEndTimeSlot()){
				return true;
			}
		}
		return false;
	}
}
