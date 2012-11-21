package nl.hu;

public class InstructionMessage
{


	public String instruction;
	public String destination;
	public String look_up;
	public Object look_up_parameters;
	public Object payload;

	public InstructionMessage(String instruction, String destination, String look_up, Object look_up_parameters ,Object payload)
	{
		this.instruction = instruction;
		this.destination = destination;
		this.look_up = look_up;
		this.look_up_parameters = look_up_parameters;
		this.payload = payload;		

	}





}