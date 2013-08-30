package pa_server;

class Servlet {
	private Request request;
	private final Response response;
	private final int id;

	public Servlet(int id) {
		this.id = id;

		this.response = new Response();
	}
	
	@Override
	public void finalize() throws Throwable {
		response.clear();
		request.clear();
	}
}
