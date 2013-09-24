package libraries.utillities.ACLMessageAnalyzer;

public class Main {

	public static void main(String[] args) {
		
		ACLMessageParser parser = new ACLMessageParser(System.getenv("MSGPATH"), System.getenv("MSGRESULT"));
		parser.analyzeData();
	}

}
