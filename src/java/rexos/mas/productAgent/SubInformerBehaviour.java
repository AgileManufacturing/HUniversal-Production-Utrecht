package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import rexos.libraries.log.Logger;
import rexos.mas.behaviours.ReceiveBehaviour;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.BehaviourStatus;

public class SubInformerBehaviour extends ReceiveBehaviour {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private AID _targetEquiplet;
	private long _timeslotDuration;
	private ProductionStep _productionStep;
	private InformerBehaviour _parentBehaviour;
	private String _conversationId;

	private static int SUBINFORMER_TIMEOUT = 10000;

	private int _currentState = 0;

	public SubInformerBehaviour(Agent myAgent,
			InformerBehaviour parentBehaviour, ProductionStep productionStep,
			AID targetEquiplet) {
		super(
				myAgent,
				SUBINFORMER_TIMEOUT,
				MessageTemplate
						.and(MessageTemplate.MatchConversationId(productionStep
								.getConversationIdForEquiplet(targetEquiplet)),
								MessageTemplate.or(
										MessageTemplate
												.MatchOntology("CanPerformStep"),
										MessageTemplate
												.MatchOntology("ProductionDuration"))));
		_parentBehaviour = parentBehaviour;
		_productionStep = productionStep;
		_targetEquiplet = targetEquiplet;
		_conversationId = _productionStep
				.getConversationIdForEquiplet(_targetEquiplet);
	}

	@Override
	public void onStart() {
		super.onStart();
		try {
			ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
			message.setConversationId(_conversationId);
			message.addReceiver(_targetEquiplet);
			message.setOntology("CanPerformStep");
			message.setContentObject(_productionStep);
			myAgent.send(message);
		} catch (Exception e) {
			Logger.log(e);
		}
	}

	@Override
	public void handle(ACLMessage message) {
		try {
			if (message != null) {
				switch (_currentState) {

				case 0:
					if (message.getPerformative() == ACLMessage.CONFIRM) {
						ACLMessage newMessage = new ACLMessage(
								ACLMessage.REQUEST);
						newMessage.setConversationId(_conversationId);
						newMessage.addReceiver(_targetEquiplet);
						newMessage.setOntology("GetProductionDuration");
						newMessage.setContentObject(_productionStep);
						myAgent.send(newMessage);
						_currentState++;
					} else {
						Logger.log("Received something different than Confirm.");
						_parentBehaviour.callbackSubInformerBehaviour(
								BehaviourStatus.ERROR, this);
					}
					break;
				case 1:
					if (message.getPerformative() == ACLMessage.INFORM) {
						this._timeslotDuration = (Long) message
								.getContentObject();
						_parentBehaviour.callbackSubInformerBehaviour(
								BehaviourStatus.COMPLETED, this);
					} else {
						Logger.log("Received something different than Inform.");
						_parentBehaviour.callbackSubInformerBehaviour(
								BehaviourStatus.ERROR, this);
					}
					break;
				default:
					break;
				}
			} else {
				Logger.log("Message can't be null!");
				_parentBehaviour.callbackSubInformerBehaviour(
						BehaviourStatus.ERROR, this);
			}
		} catch (Exception e) {
			Logger.log(e);
			_parentBehaviour.callbackSubInformerBehaviour(
					BehaviourStatus.ERROR, this);
		}
	}

	public long getTimeslotDuration() {
		return _timeslotDuration;
	}

	public AID getTargetEquiplet() {
		return _targetEquiplet;
	}

	public int getProductionStepId() {
		return _productionStep.getId();
	}

}
