/**
 * @file OverviewBehaviour.java
 * @brief Behaviour in which the product agent calls upon all subbehaviours
 * @date Created: 08-04-2013
 * 
 * @author Alexander Streng
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

package productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import newDataClasses.ProductionStep;

public class OverviewBehaviour extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;
	@SuppressWarnings("unused")
	private boolean _isDone = false;
	/* Behaviour */
	@SuppressWarnings("unused")
	private PlannerBehaviour _plannerBehaviour;
	private InformerBehaviour _informerBehaviour;
	private SchedulerBehaviour _schedulerBehaviour;
	@SuppressWarnings("unused")
	private ProduceBehaviour _produceBehaviour;
	private SequentialBehaviour _sequentialBehaviour;

	public OverviewBehaviour(){
		System.out.println("New overview behaviour created.");
	}

	/*
	 * (non-Javadoc) This behaviour should have 3 parallel sub-behaviours. One
	 * where the normal flow ( plan, inform, schedule, produce ) is followed,
	 * and one where it listens to incoming messages. MIST NOG 1 MOGELIJKHEID!
	 */
	@Override
	public void action(){
		_productAgent = (ProductAgent) myAgent;
		System.out.println("Add a SequentialBehaviour");
		_sequentialBehaviour = new SequentialBehaviour();
		_productAgent.addBehaviour(_sequentialBehaviour);
		System.out.println("Add a PlannerBehaviour");
		_sequentialBehaviour.addSubBehaviour(new PlannerBehaviour());
		System.out.println("Add an InformerBehaviour");
		_informerBehaviour = new InformerBehaviour();
		_sequentialBehaviour.addSubBehaviour(_informerBehaviour);
		// we need to wait till all conv. of the informer are done. We don't
		// want to block, but do want to wait.
		_sequentialBehaviour.addSubBehaviour(new CyclicBehaviour(){
			private static final long serialVersionUID = 1L;

			@Override
			public void action(){
				if (_informerBehaviour.isDone()){
					_sequentialBehaviour.removeSubBehaviour(this);
				}
			}
		});
		System.out.println("Add a SchedulerBehaviour");
		_schedulerBehaviour = new SchedulerBehaviour();
		_sequentialBehaviour.addSubBehaviour(_schedulerBehaviour);
		System.out.println("Add a ProduceBehaviour");
		// _produceBehaviour = new ProduceBehaviour();
		// _sequentialBehaviour.addSubBehaviour(_produceBehaviour);
		System.out
				.println("Added all behaviours. And everything should start.");
		System.out.println("Add a Scheduler");
		_schedulerBehaviour = new SchedulerBehaviour();
		_sequentialBehaviour.addSubBehaviour(_schedulerBehaviour);
		_sequentialBehaviour.addSubBehaviour(new OneShotBehaviour(){
			private static final long serialVersionUID = 1L;

			@Override
			public void action(){
				System.out.println("\n");
				for(ProductionStep stp : _productAgent.getProduct()
						.getProduction().getProductionSteps()){
					System.out.println("ProductionStep " + stp.getId()
							+ " has Equiplets;");
					for(AID aid : _productAgent.getProduct().getProduction()
							.getProductionEquipletMapping()
							.getEquipletsForProductionStep(stp.getId())
							.keySet()){
						System.out.println("Eq localname: "
								+ aid.getLocalName()
								+ " AID: "
								+ aid
								+ " timeslots: "
								+ _productAgent
										.getProduct()
										.getProduction()
										.getProductionEquipletMapping()
										.getTimeSlotsForEquiplet(stp.getId(),
												aid));
					}
					System.out.println("\n");
				}
			}
		});
		System.out.println("Add a ProduceBehaviour");
		// _produceBehaviour = new ProduceBehaviour();
		// _sequentialBehaviour.addSubBehaviour(_produceBehaviour);
	}

	@SuppressWarnings("static-method")
	public void reschedule(){
		System.out.println("Rescheduling will be implemented here");
	}
}
