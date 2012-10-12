package nl.hu.blackboard.server;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;

import nl.hu.blackboard.data.AvailabilityFilter;
import nl.hu.blackboard.data.CleanBlackboardFilter;
import nl.hu.blackboard.data.InitiateRescheduleFilter;
import nl.hu.blackboard.data.PostItFilter;
import nl.hu.blackboard.data.PostItProtos.PostIt;
import nl.hu.blackboard.data.PostItProtos.PostItBox;
//import nl.hu.lcv.blackboard.data.PostItBox;

import org.openbbs.blackboard.Blackboard;
import org.openbbs.blackboard.BlackboardAccess;
import org.openbbs.blackboard.EntryFilter;
import org.openbbs.blackboard.NamedZone;
import org.openbbs.blackboard.Zone;
import org.openbbs.blackboard.ZonedBlackboardAccess;


public class BlackboardThread implements Runnable
{
	private Socket connection;
	//private ObjectOutputStream oos;
	//private ObjectInputStream ois;
	private BufferedInputStream bis;
	private BufferedOutputStream bos;
	private Blackboard blackboard;
	private BlackboardAccess blackboardAccess;
	
	public BlackboardThread(Blackboard theBlackboard, Socket theConnection, BlackboardAccess theAccess)
	{
		this.blackboard = theBlackboard;
		this.connection = theConnection;
		this.blackboardAccess = theAccess;					
	}
	
	@Override
	public void run() 
	{
		try {    		
							
			PostItBox postItBox;		
			bis = new BufferedInputStream(connection.getInputStream());
			bos = new BufferedOutputStream(connection.getOutputStream());
			ByteArrayOutputStream baos = new ByteArrayOutputStream();
			
			byte receivedByte = 0;
			while(receivedByte != -1)
			{
				receivedByte = (byte) bis.read();
				if(receivedByte != -1)
					baos.write(receivedByte);	
			
			}		
			
			postItBox = PostItBox.parseFrom(baos.toByteArray());
			System.out.println(postItBox.getReadOwner());
				
			//(PostItBox)ois.readObject();		
			initBlackboardAccess(postItBox);										
			//might be switch
			if(postItBox.getIsWrite())							
				write(postItBox);						
			else 			
				read(postItBox);		
		} 
    	catch (IOException e) {
			e.printStackTrace();
		} 
		finally
		{					  
				//4: Closing connection
				try{
					bis.close();
					bos.close();				
					connection.close();					
				}
				catch(IOException ioException){
					ioException.printStackTrace();
				}
		}  	    		
	}
	
	public void initBlackboardAccess(PostItBox postItBox)
	{
		if(postItBox.getZone() != null)
		{
			Zone zone = new NamedZone(postItBox.getZone());		
			if(!blackboard.isZoneOpen(zone))
				blackboard.openZone(zone);		
			blackboardAccess = new ZonedBlackboardAccess(blackboard, zone);	
		}	
	}
	
	public void read(PostItBox postItBox) throws IOException
	{					
		//Do some jibberdy jabberdy to get an object out of blackboard and write back that its processed
			ArrayList<PostIt> arr = new ArrayList<PostIt>();
			
			EntryFilter theFilter = null;
			
			switch(postItBox.getFilter().getFiltername())
			{
			case("InitiateRescheduleFilter"):			
				theFilter = new InitiateRescheduleFilter(new ArrayList<String>( postItBox.getFilter().getAgentnameList()));
				break;				
			case("PostItFilter"):
				theFilter = new PostItFilter(postItBox);				
				break;				
			case("AvailabilityFilter"):
				theFilter = new AvailabilityFilter(postItBox.getFilter().getAgentnameList().get(0), postItBox.getFilter().getDeadline());
				break;
			}
			
			if(!postItBox.getZone().equals("BB1"))
			{
				while(blackboardAccess.exists(new CleanBlackboardFilter(postItBox)))
				{
					blackboardAccess.take(new CleanBlackboardFilter(postItBox));				
					System.out.println("Object deleted");
				}
			}
			while(blackboardAccess.exists(theFilter))
			{			 	
				//Probably needs a better place than this but these lines take a postit from bb and writes them back to tell its processed.
				PostIt takenFromBlackboard = (PostIt)blackboardAccess.take(theFilter);
				if(postItBox.getReadOwner().equals(takenFromBlackboard.getOwner()))
				{
					takenFromBlackboard = PostIt.newBuilder(takenFromBlackboard).setIsProcessed(true).build();
					//takenFromBlackboard.setProcessed(true);	
				
				}
					
				arr.add(takenFromBlackboard);						
			}  		
			//System.out.println(postItBox.getTheFilter().getClass().getSimpleName());
			if(!postItBox.getFilter().getFiltername().equals("InitiateRescheduleFilter"))
			{
				for(PostIt p : arr)
				{
					blackboardAccess.write(p);							
				}	
			}
			
			postItBox = PostItBox.newBuilder(postItBox).addAllPostIts(arr).build();
			byte[] array = postItBox.toByteArray();
			bos.write(array);
			bos.write(-1);
			bos.flush();//postItBox.setPostItObject(arr);
			//oos.writeObject(postItBox);
			//oos.flush();							
	}
	
	
	
	public void write(PostItBox postItBox)
	{   
		System.out.println("Blackboard write action");	
		ArrayList<PostIt> postIts = new ArrayList<PostIt>(postItBox.getPostItsList()); 
		for(PostIt postIt : postIts)
		{		
			blackboardAccess.write(postIt);				
		}							
	}   

}
