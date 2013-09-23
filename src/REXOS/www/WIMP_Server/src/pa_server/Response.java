package pa_server;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Set;

class Response extends HashMap<String, String> {
	private static final long serialVersionUID = 1L;

	private int code;
	private String reason;
	private byte[] bytes;

	public Response() {
	}
}
