//******************************************************************************
//
//                 Blackboard Server
//
//******************************************************************************
// Project:        Blackboard Server
// File:           BlackboardClientUtils.java	  
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

package nl.hu.blackboard.client;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import nl.hu.autokeystore.clientlib.AutoKeyStore;
import nl.hu.blackboard.data.PostItProtos.PostItBox;


// TODO: Auto-generated Javadoc
/**
 * The Class BlackboardClientUtils.
 */
public class BlackboardClientUtils 
{
	
	/** The auto key store client. */
	private static AutoKeyStore autoKeyStoreClient = new AutoKeyStore();
	
	/**
	 * Write to blackboard.
	 *
	 * @param postItBox the postit box
	 */
	public static void writeToBlackboard(PostItBox postItBox)
	{
		Socket clientSocket = null;		
		BufferedOutputStream bos = null;
		
		try {
			//System.out.println("Connecting....");
			clientSocket = new Socket(autoKeyStoreClient.getValue("blackboard.ip"), Integer.parseInt(autoKeyStoreClient.getValue("blackboard.port")));
			clientSocket.setSoLinger(false, 0);			
			bos = new BufferedOutputStream(clientSocket.getOutputStream());
			byte[] array = postItBox.toByteArray();
			bos.write(array);
			bos.write(-1);
			bos.flush();				
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		finally
		{
			try {
				bos.close();				
				clientSocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}										
		}	
	}

	/**
	 * Read from blackboard.
	 *
	 * @param ReadBox the read box
	 * @return the post it box
	 */
	public static PostItBox readFromBlackboard(PostItBox ReadBox)
	{
		Socket clientSocket = null;
		BufferedOutputStream bos = null;	
		BufferedInputStream bis = null;
		try {
				//System.out.println("Connecting....");
				clientSocket = new Socket(autoKeyStoreClient.getValue("blackboard.ip"), Integer.parseInt(autoKeyStoreClient.getValue("blackboard.port")));
				clientSocket.setSoLinger(false, 0);
										
				bos = new BufferedOutputStream(clientSocket.getOutputStream());
				bis = new BufferedInputStream(clientSocket.getInputStream());
				byte[] array = ReadBox.toByteArray();
				bos.write(array);
				bos.write(-1);
				bos.flush();
							
				ByteArrayOutputStream baos = new ByteArrayOutputStream();
				byte receivedByte = 0;
				while(receivedByte != -1)
				{
					receivedByte = (byte) bis.read();
					if(receivedByte != -1)
						baos.write(receivedByte);					
				}		
				
				ReadBox = PostItBox.parseFrom(baos.toByteArray());				
						
						
			} catch (UnknownHostException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
			finally
			{
				try {					
					//bos.close();
					bis.close();
					bos.close();
					clientSocket.close();
				} catch (IOException e) {
					e.printStackTrace();
				}										
			}		
			return ReadBox;
		
	}		
}
