/**
 * 
 */
package newDataClasses;

/**
 * @author Theodoor de Graaff <theodoor.degraaff@student.hu.nl>
 *
 */
public class ProductLog {

	private boolean writeToRemote;
	private boolean writeToLocal;
	
	//private SQLiteDatabaseConnection local;
	//private RemoteDatabaseConnection remote;
	
	public void add(LogMessage[] msg){
		if(writeToLocal){
			//local.insert()
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
