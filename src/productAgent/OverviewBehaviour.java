package ProductAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import newDataClasses.ProductionStep;

@SuppressWarnings("serial")
public class OverviewBehaviour extends OneShotBehaviour {
	private ProductAgent _productAgent;
	// private ThreadedBehaviourFactory _pbf;

	/* Behaviour */
	private PlannerBehaviour _plannerBehaviour;
	private InformerBehaviour _informerBehaviour;
	private SchedulerBehaviour _schedulerBehaviour;
	private ProduceBehaviour _produceBehaviour;
	private SequentialBehaviour _sequentialBehaviour;

	public OverviewBehaviour() {
		System.out.println("New overview behaviour created.");
	}

	/*
	 * (non-Javadoc) This behaviour should have 2 parallel sub-behaviours. One
	 * where the normal flow ( plan, inform, schedule, produce ) is follow, and
	 * one where is listend for incoming msgs.
	 */
	@Override
	public void action() {
		_productAgent = (ProductAgent) myAgent;
		
		System.out.println("Lets add a Sequential");
		_sequentialBehaviour = new SequentialBehaviour();

		System.out.println("Lets add a plannerbehaviour");
		_plannerBehaviour = new PlannerBehaviour();
		_sequentialBehaviour.addSubBehaviour(_plannerBehaviour);

		System.out.println("Lets add a Informer");
		_informerBehaviour = new InformerBehaviour();
		_sequentialBehaviour.addSubBehaviour(_informerBehaviour);
		
		_sequentialBehaviour.addSubBehaviour(new OneShotBehaviour(){
			@Override
			public void action() {
				for (ProductionStep stp : _productAgent.getProduct().getProduction().getProductionSteps()) {
					System.out.println("ProductionStep " + stp.getId() + " has Equiplets; \n");	
					for (AID aid : _productAgent.getProduct().getProduction()
							.getProductionEquipletMapping()
							.getEquipletsForProductionStep(stp.getId())) {
						System.out.println("Eq localname: " + aid.getLocalName() + " AID: " + aid + "\n");	
					}
				}			
			}
		});

		System.out.println("Lets add a Scheduler");
		//_schedulerBehaviour = new SchedulerBehaviour();
		//_sequentialBehaviour.addSubBehaviour(_schedulerBehaviour);

		System.out.println("Lets add a produce");
		//_produceBehaviour = new ProduceBehaviour();
		//_sequentialBehaviour.addSubBehaviour(_produceBehaviour);

		System.out
				.println("Added all behaviours. And everything should start. Aw yeah!");

		myAgent.addBehaviour(_sequentialBehaviour);
	}
}
