/*****************************************************************
JADE - Java Agent DEvelopment Framework is a framework to develop 
multi-agent systems in compliance with the FIPA specifications.
Copyright (C) 2002 TILAB

GNU Lesser General Public License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, 
version 2.1 of the License. 

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*****************************************************************/

package wsig.agent;

import jade.content.lang.sl.SLCodec;
import jade.content.onto.Ontology;
import jade.core.AID;
import jade.domain.DFService;
import jade.domain.FIPANames;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.FIPAManagementOntology;
import jade.domain.FIPAAgentManagement.Property;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.domain.JADEAgentManagement.JADEManagementOntology;
import jade.lang.acl.ACLMessage;
import jade.proto.SubscriptionInitiator;
import jade.wrapper.gateway.GatewayAgent;

import java.util.Iterator;

import org.apache.log4j.Logger;
import org.uddi4j.util.ServiceKey;

import wsig.WSIGConfiguration;
import wsig.WSIGConstants;
import wsig.store.WSIGService;
import wsig.store.WSIGStore;
import wsig.uddi.UDDIManager;
import wsig.wsdl.JadeToWSDL;
import wsig.wsdl.WSDLConstants;


public class WSIGAgent extends GatewayAgent implements WSIGConstants {

	private static final long serialVersionUID = 3815496986569126415L;

	private static Logger log = Logger.getLogger(WSIGAgent.class.getName());

	private WSIGStore wsigStore = null;
	private UDDIManager uddiManager = null;

	protected void setup() {
		super.setup();

		// Set non-standard archive scheme called wsjar" 
		System.setProperty("org.eclipse.emf.common.util.URI.archiveSchemes", "wsjar wszip jar zip");

		// Set soap message factory library (solve jboss problem)
		System.setProperty("javax.xml.soap.MessageFactory", "org.apache.axis.soap.MessageFactoryImpl"); 
		
		log.info("Agent "+getLocalName()+" - starting...");

		// Get agent arguments
		Object[] args = getArguments();
		for (int i = 0; i < args.length; i++) {
			log.info("arg[" + i + "]" + args[i]);
		}

		// Verify if config file is passed as first agent parameter
		String confFile = null; 
		if (args.length >= 1 && !("".equals((String)args[0]))) {
			confFile = (String)args[0];
		}

		// Init agent configuration
		WSIGConfiguration.init(confFile);

		// Verify if wsdlDirectory is passed as second agent parameter
		if (args.length >= 2 && !("".equals((String)args[1]))) {
			String wsdlDirectory = (String)args[1];
			WSIGConfiguration.getInstance().setWsdlDirectory(wsdlDirectory);
		}
		
		// Verify if wsigStore is passed as third agent parameter
		if (args.length >= 3 && (args[2] instanceof WSIGStore)) {
			wsigStore = (WSIGStore)args[2];
		}
		if (wsigStore == null) {
			wsigStore = new WSIGStore();
		}

		// Create UDDIManager
		if (WSIGConfiguration.getInstance().isUddiEnable()) {
			uddiManager = new UDDIManager();
		}

		// Register ontology & language
		getContentManager().registerOntology(FIPAManagementOntology.getInstance());
		getContentManager().registerOntology(JADEManagementOntology.getInstance());
		getContentManager().registerLanguage(new SLCodec());

		// Register into a DF
		registerIntoDF();

		// Subscribe to the DF
		DFAgentDescription template = new DFAgentDescription();
		ServiceDescription sd = new ServiceDescription();
		sd.addProperties(new Property(WSIG_FLAG, "true"));
		template.addServices(sd);
		ACLMessage subscriptionMsg = DFService.createSubscriptionMessage(this, getDefaultDF(), template, null);
		addBehaviour(new SubscriptionInitiator(this, subscriptionMsg) {

			protected void handleInform(ACLMessage inform) {
				log.debug("Agent "+getLocalName()+" - Notification received from DF ("+inform.getContent()+")");
				try {
					DFAgentDescription[] dfds = DFService.decodeNotification(inform.getContent());
					for (int i = 0; i < dfds.length; ++i) {
						Iterator services = dfds[i].getAllServices();
						if (services.hasNext()) {
							// Registration of an agent
							registerAgent(dfds[i]);
						} else {
							// Deregistration of an agent
							deregisterAgent(dfds[i]);
						}
					}
				}
				catch (Exception e) {
					log.warn("Agent "+myAgent.getLocalName() + " - Error processing DF notification", e);
				}
			}
		});

		log.info("Agent "+getLocalName()+" - started!");
	}

	public WSIGStore getWSIGStore() {
		return wsigStore;
	}

	private synchronized void registerAgent(DFAgentDescription dfad) throws Exception {

		log.info("Start wsigs's registration from agent: " + dfad.getName());

		// Loop all services of agent
		ServiceDescription sd;
		WSIGService wsigService;

		AID agentId = dfad.getName();
		Iterator it = dfad.getAllServices();
		while (it.hasNext()) {
			sd = (ServiceDescription) it.next();

			// Create wsdl & wsig service
			wsigService = createWSIGService(agentId, sd);

			// Register new service
			registerService(wsigService);
		}

		log.info("End wsigs's registration from agent: " + dfad.getName());
	}

	private void registerService(WSIGService wsigService) throws Exception {

		if (null != wsigService) {
			log.info("Create new wsig service: "+wsigService.toString());

			// Register wsigService into UDDI
			if (uddiManager != null) {
				ServiceKey uddiServiceKey = uddiManager.UDDIRegister(wsigService);
				wsigService.setUddiServiceKey(uddiServiceKey);
			}

			// Store wsigService into WSIGStore
			wsigStore.addService(wsigService.getServiceName(), wsigService);
		}
	}

	private synchronized void deregisterAgent(DFAgentDescription dfad) throws Exception {

		log.info("Start wsigs's deregistration from agent: " + dfad.getName());

		WSIGService wsigService;

		AID agentId = dfad.getName();
		Iterator<WSIGService> it = wsigStore.getServices(agentId).iterator();
		while (it.hasNext()) {
			wsigService = it.next();

			// Deregister service
			deregisterService(wsigService);
		}

		log.info("End wsigs's deregistration from agent: " + dfad.getName());
	}

	private void deregisterService(WSIGService wsigService) throws Exception {

		String serviceName = wsigService.getServiceName();

		log.info("Remove wsig service "+serviceName);

		// DeRegister wsigService from UDDI
		try {
			if (uddiManager != null) {
				uddiManager.UDDIDeregister(wsigService);
			}
		} catch (Exception e) {
			log.warn("Error removing service from UDDI", e);
		}

		// Remove wsigService from WSIGStore
		wsigStore.removeService(serviceName);
	}

	private boolean isWSIGService(ServiceDescription sd) {

		Property p = null;
		Iterator it = sd.getAllProperties();
		while (it.hasNext()) {
			p = (Property) it.next();

			if (WSIGConstants.WSIG_FLAG.equalsIgnoreCase(p.getName()) && p.getValue().toString().equals("true")) {
				return true;
			}
		}
		return false;
	}

	private String getServicePrefix(ServiceDescription sd) {

		Property p = null;
		Iterator it = sd.getAllProperties();
		while (it.hasNext()) {
			p = (Property) it.next();

			if (WSIGConstants.WSIG_PREFIX.equalsIgnoreCase(p.getName()) && !p.getValue().toString().equals("")) {
				return p.getValue().toString()+WSDLConstants.SEPARATOR;
			}
		}
		return "";
	}

	private WSIGService createWSIGService(AID aid, ServiceDescription sd) throws Exception {

		// Get service prefix & name
		String servicePrefix = getServicePrefix(sd);
		String serviceName = servicePrefix + sd.getName();

		// Verify if is a wsig service
		if (!isWSIGService(sd)) {
			log.info("Service "+serviceName+" discarded (no wsig service)");
			return null;
		}

		// Verify if the service is already registered
		if (wsigStore.isServicePresent(serviceName)) {
			log.info("Service "+serviceName+" of agent "+aid.getName()+" is already registered");
			return null;
		}

		// Get ontology
		// FIX-ME elaborate only first ontology
		String ontoName = null;
		Iterator ontoIt = sd.getAllOntologies();
		if (ontoIt.hasNext()) {
			System.out.println("Ontology " + ontoIt.toString() + " added");
			ontoName = (String)ontoIt.next();
		}
		if (ontoName == null) {
			log.info("Service "+serviceName+" of agent "+aid.getName()+" have not ontology registered");
			return null;
		}

		// Create ontology instance
		String ontoClassname = WSIGConfiguration.getInstance().getOntoClassname(ontoName);
		if (ontoClassname == null) {
			log.warn("Ontology "+ontoName+" not present in WSIG configuration file");
			return null;
		} else {
			System.out.println("ontoClassName = " + ontoClassname);
		}

		Ontology serviceOnto = null;
		try {
			Class ontoClass = Class.forName(ontoClassname);
			serviceOnto = (Ontology)ontoClass.newInstance();
		} catch (Exception e) {
			log.warn("Ontology class "+ontoClassname+" not present in WSIG classpath", e);
			return null;
		}
		// Register new onto in agent
		getContentManager().registerOntology(serviceOnto);
		
		// Get mapper class
		Class mapperClass = getMapperClass(sd);

		// Create new WSIGService
		WSIGService wsigService = new WSIGService();
		wsigService.setServiceName(serviceName);
		wsigService.setServicePrefix(servicePrefix);
		wsigService.setAid(aid);
		wsigService.setOnto(serviceOnto);
		wsigService.setMapperClass(mapperClass);

		// Create wsdl
		JadeToWSDL.createWSDLFromSD(this, sd, wsigService);
		System.out.println(wsigService.getServiceName());
		System.out.println(wsigService.getServicePrefix());
		
		return wsigService;
	}

	public Class getMapperClass(ServiceDescription sd) throws Exception{
		Property p = null;
		boolean mapperFound = false;
		String mapperClassName = null;

		Iterator it = sd.getAllProperties();
		while (it.hasNext() && !mapperFound) {
			p = (Property) it.next();
			if (WSIGConstants.WSIG_MAPPER.equalsIgnoreCase(p.getName())) {
				mapperClassName = (String)p.getValue();
				mapperFound = true;
			}
		}

		if(!mapperFound) {
			return null;
		}

		Class mapperClass = null;
		try {
			mapperClass = Class.forName(mapperClassName);
		} catch (ClassNotFoundException e) {
			throw new Exception("Class "+mapperClassName+" not found!", e);
		}

		return mapperClass;
	}

	private void registerIntoDF() {
		DFAgentDescription dfad = new DFAgentDescription();
		dfad.setName(this.getAID());
		dfad.addLanguages(FIPANames.ContentLanguage.FIPA_SL);
		dfad.addProtocols(FIPANames.InteractionProtocol.FIPA_REQUEST);
		ServiceDescription sd = new ServiceDescription();
		sd.setType(AGENT_TYPE);
		sd.setName(getLocalName());
		dfad.addServices(sd);
		try {
			DFService.register(this, dfad);
		} catch (Exception e) {
			log.error("Agent "+getLocalName()+" - Error during DF registration", e);
		}
	}

	protected void takeDown() {

		// Deregister all service
		try {
			WSIGService wsigService;
			Iterator<WSIGService> it = wsigStore.getAllServices().iterator();
			while(it.hasNext()) {
				wsigService = it.next();
				deregisterService(wsigService);
			}
		} catch (Exception e) {
			log.error("Agent "+getLocalName()+" - Error during service deregistration", e);
		}

		// Deregister WSIG agent
		try {
			DFService.deregister(this, getDefaultDF());
		} catch (Exception e) {
			log.error("Agent "+getLocalName()+" - Error during DF deregistration", e);
		}

		log.info("Agent "+getLocalName()+" - Taken down now");
	}
}
