/**
 * @file OverviewBehaviour.java
 * @brief Behaviour in which the product agent calls upon all subbehaviours
 * @date Created: 08-04-2013
 * 
 * @author Alexander Streng
 * @author Mike Schaap
 * @author Arno Derks
 * 
 *         Copyright © 2013, HU University of Applied Sciences Utrecht. All
 *         rights reserved.
 * 
 *         Redistribution and use in source and binary forms, with or without
 *         modification, are permitted provided that the following conditions
 *         are met: - Redistributions of source code must retain the above
 *         copyright notice, this list of conditions and the following
 *         disclaimer. - Redistributions in binary form must reproduce the above
 *         copyright notice, this list of conditions and the following
 *         disclaimer in the documentation and/or other materials provided with
 *         the distribution. - Neither the name of the HU University of Applied
 *         Sciences Utrecht nor the names of its contributors may be used to
 *         endorse or promote products derived from this software without
 *         specific prior written permission.
 * 
 *         THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *         "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *         LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *         A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *         UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *         INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *         (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *         SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *         HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *         STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *         IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *         POSSIBILITY OF SUCH DAMAGE.
 **/

package rexos.mas.productAgent;

import rexos.mas.data.AgentStatus;
import rexos.mas.data.BehaviourStatus;
import jade.core.Agent;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;

public class OverviewBehaviour extends OneShotBehaviour implements
		BehaviourCallback {

	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;

	/* Behaviour */
	private PlannerBehaviour _plannerBehaviour;
	private InformerBehaviour _informerBehaviour;
	private SchedulerBehaviour _schedulerBehaviour;
	private ProduceBehaviour _produceBehaviour;
	private SequentialBehaviour _sequentialBehaviour;
	private SocketBehaviour _socketBehaviour;

	public OverviewBehaviour(Agent myAgent) {
		super(myAgent);
		System.out
				.println("Overview behaviour created. Starting all behaviours to the agents.");
	}

	private void initialize() {
		System.out.println("Creating the SocketBehaviour");
		_socketBehaviour = new SocketBehaviour(myAgent, _productAgent
				.getProperties().getCallback());

		System.out.println("Creating the PlannerBehaviour");
		_plannerBehaviour = new PlannerBehaviour(myAgent, this);

		System.out.println("Creating the InformerBehaviour");
		_informerBehaviour = new InformerBehaviour(myAgent, this);

		System.out.println("Creating the ScheduleBehaviour");
		_schedulerBehaviour = new SchedulerBehaviour(myAgent, this);

		System.out.println("Creating the ProductBehaviour");
		_produceBehaviour = new ProduceBehaviour(myAgent, this);

		System.out.println("Creating the SequentialBehaviour");
		_sequentialBehaviour = new SequentialBehaviour();
		// Starting a sequentialBehaviour
		System.out.println("Add a SequentialBehaviour");
		_productAgent.addBehaviour(_sequentialBehaviour);

		// Starting the SocketBehaviour, so the agent can communicate with the
		System.out.println("Add a SocketBehaviour");
		_productAgent.addBehaviour(_socketBehaviour);
	}

	/*
	 * (non-Javadoc) This behaviour should have 3 parallel sub-behaviours. One
	 * where the normal flow ( plan, inform, schedule, produce ) is followed,
	 * and one where it listens to incoming messages. MIST NOG 1 MOGELIJKHEID!
	 */
	@Override
	public void action() {
		try {
			_productAgent = (ProductAgent) myAgent;
			// Initialize all behaviours
			this.initialize();
			// Star the planning behaviour. This behaviour will call the
			// handleCallback function when done,
			// which in turn create all the other behaviours
			this.startPlanning();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@SuppressWarnings("static-method")
	public void reschedule() {
		System.out.println("Rescheduling will be implemented here");
	}

	public void startPlanning() {
		_productAgent.setStatus(AgentStatus.PLANNING);
		System.out.println("Add a PlannerBehaviour");
		_sequentialBehaviour.addSubBehaviour(_plannerBehaviour);
	}

	public void startInforming() {
		_productAgent.setStatus(AgentStatus.INFORMING);
		System.out.println("Add an InformerBehaviour");
		_sequentialBehaviour.addSubBehaviour(_informerBehaviour);
	}

	public void startScheduling() {
		_productAgent.setStatus(AgentStatus.SCHEDULING);
		System.out.println("Add a SchedulerBehaviour");
		_sequentialBehaviour.addSubBehaviour(_schedulerBehaviour);
	}

	public void startProducing() {
		_productAgent.setStatus(AgentStatus.PRODUCING);
		System.out.println("Add a ProduceBehaviour");
		_sequentialBehaviour.addSubBehaviour(_produceBehaviour);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.productAgent.BehaviourCallback#handleCallback()
	 */
	@Override
	public void handleCallback(BehaviourStatus bs) {
		// USE behaviourstatus to check if behaviour finished nicely
		// TODO Auto-generated method stub
		AgentStatus as = _productAgent.getStatus();
		switch (as) {
		case PLANNING:
			System.out.println("Done planning.");
			this.startInforming();
			break;
		case INFORMING:
			System.out.println("Done Informing");
			this.startScheduling();
			break;
		case SCHEDULING:
			System.out.println("Done scheduling");
			this.startProducing();
			break;
		case PRODUCING:
			System.out.println("DONE!.");
			this.cleanBehaviour();
			break;
		default:
			System.out.println("Unknown status. Status: " + as.toString());
			break;
		}
	}

	public void cleanBehaviour() {
		_socketBehaviour.stop();
	}
}
