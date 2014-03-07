/**
 * @file src/REXOS/MAS/agents/hardware_agent/behaviours/RequiredModulesPresent.java
 * @brief Handles the RequiredModulesPresent message
 * @date Created: 12-04-13
 * 
 * @author Thierry Gerritse
 * @author Jan-Willem Willebrands
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * @note 2013-05-14 JWW: This should be adapted to use MOST once it's done.
 **/
package agents.hardware_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.util.ArrayList;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Queries;
import libraries.knowledgedb_client.Row;
import agents.hardware_agent.HardwareAgent;
import agents.shared_behaviours.ReceiveBehaviour;

/**
 * Class for the receivebehaviour to receive messages with the ontology RequiredModulesPresent
 */
public class RequiredModulesPresent extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID.
	 **/
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate MESSAGE_TEMPLATE
	 *      Contains the MessageTemplate to match.
	 */
	private static final MessageTemplate MESSAGE_TEMPLATE = MessageTemplate.MatchOntology("RequiredModulesPresent");

	/**
	 * @var HardwareAgent hardwareAgent
	 *      The hardwareAgent for this behaviour.
	 */
	private HardwareAgent hardwareAgent;

	/**
	 * Instantiates a new check for module.
	 * 
	 * @param hardwareAgent
	 * 		  The hardwareagent of this behaviour.
	 */
	public RequiredModulesPresent(HardwareAgent hardwareAgent) {
		super(hardwareAgent, MESSAGE_TEMPLATE);
		this.hardwareAgent = hardwareAgent;
	}

	/**
	 * Returns a list of module group ids for which a module is available on this equiplet. This data is currently
	 * retrieved from the knowledge base. Once MOST is implemented, this method should take MOST data into account.
	 * 
	 * @return An arraylist containing module group ids for the modules that are attached to this equiplet.
	 **/
	private ArrayList<Integer> getAvailableModuleGroups() {
		ArrayList<Integer> availableModules = new ArrayList<Integer>();
		try {
			// get the availablemoduleGroups
			KnowledgeDBClient client = new KnowledgeDBClient();
			Row[] rows =
					client.executeSelectQuery(Queries.MODULES_PER_EQUIPLET, hardwareAgent.getEquipletAgentAID()
							.getLocalName());
			for(Row row : rows) {
				availableModules.add((Integer) row.get("groupId"));
			}
		} catch(KeyNotFoundException | KnowledgeException ex) {
			// TODO:(Check this)Return the current (possibly empty) arraylist if reading from the knowledge db fails for
			// whatever reason.
		}

		return availableModules;
	}

	/**
	 * Responds to incoming messages querying whether a module is available for a set of module group ids. This method
	 * will respond with a RequiredModulesPresent message. If all modules are available, a CONFIRM will be sent. If one
	 * or more modules
	 * are missing, a DISCONFIRM will be sent.
	 * 
	 * @param message The incoming message.
	 */
	@Override
	public void handle(ACLMessage message) {
		
		if ( message.getPerformative() == ACLMessage.QUERY_IF){
			boolean modulesPresent = true;
			try {
				// check if modules are present
				int[] moduleGroupIds = (int[]) message.getContentObject();
				ArrayList<Integer> availableModuleGroups = getAvailableModuleGroups();
	
				for(int groupId : moduleGroupIds) {
					if(!availableModuleGroups.contains(groupId)) {
						modulesPresent = false;
						break;
					}
				}
			} catch(UnreadableException ex) {
				modulesPresent = false;
			} finally {
				// send reply; confirm if modules are present, disconfirm if not.
				ACLMessage reply;
				reply = message.createReply();
				reply.setOntology("RequiredModulesPresent");
				if(modulesPresent) {
					reply.setPerformative(ACLMessage.CONFIRM);
				} else {
					reply.setPerformative(ACLMessage.DISCONFIRM);
				}
	
				hardwareAgent.send(reply);
			}
		}
	}
}
