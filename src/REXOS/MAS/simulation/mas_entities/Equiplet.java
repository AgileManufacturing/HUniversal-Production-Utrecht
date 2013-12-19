/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	Equiplet.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-17
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Roy Scheefhals
 *   .dMMMMMF           7Y=d9  dMMMMMr    	@author	Alexander Streng
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	@section LICENSE
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    	License:	newBSD
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		Copyright � 2013, HU University of Applied Sciences Utrecht. 
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  	All rights reserved.
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM     
 *                                        
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

package simulation.mas_entities;

import java.util.Arrays;
import java.util.Date;
import java.util.StringTokenizer;

import simulation.Updatable;
import simulation.data.Capability;
import simulation.data.TimeSlot;

public class Equiplet implements Updatable{

	private Capability[] capabilities;
	
	
	
	public Equiplet(String[][] csvArguments){
		parseEquipletCSV(csvArguments);
	}
	
	public Equiplet(Capability[] capabilities) {
		this.capabilities = capabilities;
	}
	
	public void getFreeTimeSlots(){
		
	}
	
	public boolean canPerformStep(Capability capability){
		return Arrays.asList(capabilities).contains(capability);
	}
	
	public double getLoad(TimeSlot timeSlot){
		// TODO do something 
		return 0.5;
	}
	
	public TimeSlot getFirstFreeTimeSlot(long currentTimeSlot, long duration){
		return null;
	}
	
	public boolean isScheduleLocked(){
		return true;
	}
	
	public void schedule(ProductStep step, TimeSlot timeslot){
		
	}
	
	public void removeFromSchedule(ProductStep step){
		
	}
	
	private void parseEquipletCSV(String[][] fields){
		
	}

	@Override
	public void update(long time) {
		// TODO Auto-generated method stub
		
	}
}
