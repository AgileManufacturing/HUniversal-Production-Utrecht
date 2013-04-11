/**
 * 
 */
package newDataClasses;

import newDataClasses.sqldatadase.sqliteDatabase;

/**
 * @author Theodoor de Graaff <theodoor.degraaff@student.hu.nl>
 *
 */
public class ProductLog {

	private boolean writeToRemote;
	private boolean writeToLocal = true; 
	
	private sqliteDatabase local;
	//private RemoteDatabaseConnection remote;
	
	public void add(LogMessage[] msg){
		if(writeToLocal){
			local.insert(msg);
		}
		if(writeToRemote){
			//remote.insert()
		}
	}
	
	public void pushLocalToRemote(){
		//get latest remote
		//get local since latest remote
		//write to remote
	}
	
}
