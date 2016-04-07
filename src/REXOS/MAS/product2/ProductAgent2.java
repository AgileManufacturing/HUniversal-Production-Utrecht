package MAS.product;

import jade.core.AID;
import jade.core.Agent;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPANames;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;

import org.json.JSONException;

import MAS.util.Tick;

public class ProductAgent2 extends Agent {
	private static final long serialVersionUID = -7419809733484399023L;
	private Tick created;

	public void setup() {
		Object[] args = getArguments();
		this.created = new Tick();
        
        System.out.println("PA:" + getLocalName() + " created!");
	}

	/**
	 * @return time of agent creation
	 */
	protected Tick getCreated() {
		return created;
	}

	@Override
	public String toString() {
		return "ProductAgent2";
	}
}
