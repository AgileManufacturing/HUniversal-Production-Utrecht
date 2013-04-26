/**
 * 
 */

package newDataClasses;

import java.util.List;

import newDataClasses.sqldatabase.sqliteDatabase;

/**
 * @author Theodoor de Graaff <theodoor.degraaff@student.hu.nl>
 * 
 */
public class ProductLog{
	private boolean writeToRemote;
	private boolean writeToLocal = true;
	private sqliteDatabase local;

	// private RemoteDatabaseConnection remote;
	public void add(List<LogMessage> msgs){
		if (writeToLocal){
			local.insert(msgs);
		}
		if (writeToRemote){
			// remote.insert()
		}
	}

	/**
	 * @param writeToRemote
	 * @param writeToLocal
	 * @param local
	 */
	public ProductLog(boolean writeToRemote, boolean writeToLocal,
			sqliteDatabase local){
		super();
		this.writeToRemote = writeToRemote;
		this.writeToLocal = writeToLocal;
		this.local = local;
	}

	public void pushLocalToRemote(){
		// TODO:
		// get latest remote
		// get local since latest remote
		// write to remote
	}
}
