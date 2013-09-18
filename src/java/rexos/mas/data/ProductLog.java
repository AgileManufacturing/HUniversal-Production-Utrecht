/**
 * @file rexos/mas/data/ProductLog.java
 * @brief Class for logging productiondata
 * @date Created: 02-04-2013
 * 
 * @author Theodoor de Graaff
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

package rexos.mas.data;

import jade.core.AID;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.concurrent.atomic.AtomicLong;

import rexos.utillities.log.LogLevel;
import rexos.utillities.log.Logger;

import com.mongodb.BasicDBObject;

/**
 * @author Theodoor
 * 
 */
public class ProductLog{
	File logfile;
	FileWriter writer;
	
	public ProductLog(){
	}

	/**
	 * Add statusdata to ProductLog Writes multiple json-objects into one large
	 * json-object with an objectnumber.
	 * 
	 * @param aid
	 * @param statusData
	 * 
	 */
	public void add(AID aid, BasicDBObject statusData){
		try{
			boolean newFileCreated = false;
			if (logfile == null){
				logfile = new File("log " + aid.toString().replaceAll("[\\/:*?\"<>|]", "") + ".json");
				logfile.createNewFile();
				newFileCreated = true;
			}
			if (writer == null){
				writer = new FileWriter(logfile, true);
			}
			if (newFileCreated){
				writer.append("{\"" + uniqueCurrentTime() + "\":");
				writer.flush();
			} else{
				try(RandomAccessFile raf = new RandomAccessFile(logfile, "rw")){
					logfile.length();
					raf.setLength((logfile.length() - 1));
					raf.close();
				}
				String newObject = new String(",\"" + uniqueCurrentTime()
						+ "\":");
				writer.append(newObject);
				writer.flush();
			}
			writer.append(statusData.toString() + "}");
			writer.flush();
		} catch(IOException e){
			Logger.log(LogLevel.ERROR, e);
		}
	}
	
	private static final AtomicLong last_time = new AtomicLong();	
	/**
	 * Returns uniqueCurrentTime
	 * @see http://stackoverflow.com/questions/9191288
	 */
	public static long uniqueCurrentTime() {
	    long newCurrentTime = System.currentTimeMillis();
	    while(true) {
	        long lastTime = last_time.get();
	        if (lastTime >= newCurrentTime)
	        	newCurrentTime = lastTime+1;
	        if (last_time.compareAndSet(lastTime, newCurrentTime))
	            return newCurrentTime;
	    }
	}
}
