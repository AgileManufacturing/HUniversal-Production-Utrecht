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
