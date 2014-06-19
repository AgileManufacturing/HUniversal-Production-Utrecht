package agents.equiplet_agent.reconfigure.behaviours;

import java.net.Inet4Address;
import java.net.UnknownHostException;

import javax.jws.WebParam;
import javax.jws.WebService;
import javax.xml.ws.Endpoint;

import agents.equiplet_agent.reconfigure.ModuleDataManager;
import agents.equiplet_agent.reconfigure.datatypes.ModuleTree;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import HAL.HardwareAbstractionLayer;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;

import jade.core.behaviours.Behaviour;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KnowledgeException;

public class ReconfigureBehaviour extends Behaviour{
	int i = 0;
	public HardwareAbstractionLayer HAL;
	public ReconfigureBehaviour(HardwareAbstractionLayer hal) {
		this.HAL = hal;
	}

	@Override
	public void action() {
		// TODO Auto-generated method stub
		if(i == 0){
	        try {
				QrReceiver qr = new QrReceiver();

				Endpoint.publish("http://" + Inet4Address.getLocalHost().getHostAddress() + 
							":9191/QrReceiver", qr);
				System.out.println("http://" + Inet4Address.getLocalHost().getHostAddress() + 
						":9191/QrReceiver published");
			} catch (UnknownHostException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (KnowledgeException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	        
		}i++;
	}

	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return false;
	}
	
	@WebService
	public class QrReceiver {
		
		 /**
		  * @var moduleTree
		  * The object is being used to create a moduleTree based on the bottomModules from the HAL.
		  */
		private ModuleTree moduleTree;
		
		
		 /**
		  * getModules()
		  * Recieving all the bottomModules from HAL.
		  * Send the bottomModules to the moduleTree to create a Tree.
		  * @return String[][] array that contains the information about all the modules that the equiplet contains.
		 * @throws JarFileLoaderException 
		 * @throws FactoryException 
		  */
		public String[][] getModules() throws FactoryException, JarFileLoaderException {
			moduleTree = new ModuleTree(HAL.getBottomModules());
			
			return moduleTree.getModuleTree();
		}

		 /**
		  * addModule(String)
		  * Adding a new module to the equiplet.
		  * @param The module name as a string type.
		  * The module name is being used to retrieve the right data.
		  * This is done by the ModuleDataManager.
		  * The returning string data from the ModuleDataManager is casted to a JSON object and then send to the HAL.
		  */
		public void addModule(@WebParam(name="moduleDataJson") String moduleDataJson) {
			try {
				if(moduleDataJson != null) {
					JsonObject moduleSettings = new JsonParser().parse(moduleDataJson).getAsJsonObject();
					ModuleDataManager moduleDataManager = new ModuleDataManager(moduleSettings.get("qrCode").getAsString());				
					JsonObject staticSettings = new JsonParser().parse(moduleDataManager.getJsonFileAsString()).getAsJsonObject();
					JsonObject dynamicSettings = new JsonObject();
					dynamicSettings.add("attachedTo", moduleSettings.get("attachedTo"));
					dynamicSettings.add("mountPointX", moduleSettings.get("mountPointX"));
					dynamicSettings.add("mountPointY", moduleSettings.get("mountPointY"));
					System.out.println("Static: "+staticSettings.toString());
					System.out.println("Static: "+dynamicSettings.toString());
					HAL.insertModule(staticSettings, dynamicSettings);
				}
			} catch (Exception e) {
				e.printStackTrace();
			}

		}
		 /**
		  * editModule(String)
		  * Editing a existing module.
		  * @param The module name as a string type.
		  * The module name is being used to let the HAL know what module needs to be updated.
		  */
		public void editModule(@WebParam(name="moduleDataJson") String moduleDataJson) {
			/*
			 * @TODO
			 * Change the Module information
			 * Send the information to the HAL
			 */
		}
		
		
		 /**
		  * removeModule(String)
		  * removeModule a  module from the equiplet.
		  * @param The module name as a string type.
		  * The module name is being used to retrieve the right from the HAL.
		  * The returning data from the HAL is written away to the USB file.
		  * This is done by the ModuleDataManager.
		  */
		public void removeModule(@WebParam(name="moduleDataJson") String moduleDataJson) {

			try {
				if(moduleDataJson != null) {
					JsonObject moduleSettings = new JsonParser().parse(moduleDataJson).getAsJsonObject();
					String[] splittedQrString = moduleSettings.get("qrCode").getAsString().split("\\|");				

					ModuleIdentifier moduleIdentifier = new ModuleIdentifier(splittedQrString[1], splittedQrString[2], splittedQrString[3]);
					JsonObject moduleData = HAL.deleteModule(moduleIdentifier);
					if(moduleData != null){
						ModuleDataManager moduleDataManager = new ModuleDataManager(moduleSettings.get("qrCode").getAsString());
						moduleDataManager.writeStringToJsonFile(moduleData.toString());			
					}
				}

			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

}
