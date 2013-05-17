
package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import rexos.mas.data.ProductionStep;

public class OverviewBehaviour extends OneShotBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;
	@SuppressWarnings("unused")
	private boolean _isDone;
	// private ThreadedBehaviourFactory _pbf;
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
	 * where the normal flow ( plan, inform, schedule, produce ) is follow, and
	 * one where is listend for incoming msgs.
	 */
	@Override
	public void action(){
		_productAgent = (ProductAgent) myAgent;
		System.out.println("Lets add a Sequential");
		_sequentialBehaviour = new SequentialBehaviour();
		_productAgent.addBehaviour(_sequentialBehaviour);
		System.out.println("Lets add a plannerbehaviour");
		_sequentialBehaviour.addSubBehaviour(new PlannerBehaviour());
		System.out.println("Lets add a Informer");
		_informerBehaviour = new InformerBehaviour();
		_sequentialBehaviour.addSubBehaviour(_informerBehaviour);
		// we need to wait till all conv. of the informer are done. We dont want
		// to block, but do want to wait.
		_sequentialBehaviour.addSubBehaviour(new CyclicBehaviour(){
			private static final long serialVersionUID = 1L;

			@Override
			public void action(){
				// TODO Auto-generated method stub
				if (_informerBehaviour.isDone()){
					_sequentialBehaviour.removeSubBehaviour(this);
				}
			}
		});

		System.out.println("Lets add a Scheduler");
		 _schedulerBehaviour = new SchedulerBehaviour();
		 _sequentialBehaviour.addSubBehaviour(_schedulerBehaviour);
		 
		System.out.println("Lets add a produce");
		// _produceBehaviour = new ProduceBehaviour();
		// _sequentialBehaviour.addSubBehaviour(_produceBehaviour);

		System.out.println("Added all behaviours. And everything should start. Aw yeah!");
		System.out.println("Lets add a Scheduler");
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
		System.out.println("Lets add a produce");
		// _produceBehaviour = new ProduceBehaviour();
		// _sequentialBehaviour.addSubBehaviour(_produceBehaviour);
	}

	public void reschedule(){
		System.out.println("Rescheduling will be implemented here");
	}
}
