package servlets;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.util.ArrayList;
import javax.servlet.http.HttpServletRequest;

import org.apache.catalina.websocket.MessageInbound;
import org.apache.catalina.websocket.StreamInbound;
import org.apache.catalina.websocket.WebSocketServlet;
import org.apache.catalina.websocket.WsOutbound;

import pa_server.PACommunicationServer;

import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;

import data.CommandContainer;
import data.LoginData;

public class ProductServlet extends WebSocketServlet {
	private static final long serialVersionUID = 1L;
	private static ArrayList<MyMessageInbound> mmiList = new ArrayList<MyMessageInbound>();
	public PACommunicationServer pacs;
	
	public void writeToXML(String s, MyMessageInbound mmib){
		
		long unixTime = System.currentTimeMillis() / 1000L;
		
		PrintWriter writer;
		try {
			writer = new PrintWriter("product-" + unixTime + ".xml", "UTF-8");
			writer.println(s);
			writer.close();
			String message = "Wrote XML to: " + "product-" + unixTime + ".xml";
			System.out.println(message);
			mmib.sendMessage("{'error': false, 'message':'" + message + "'}");
						
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	public void sendToGWServer(CharBuffer s, String ip, int port,
			MyMessageInbound mmib) {

		Socket socket = null;
		DataOutputStream os = null;
		DataInputStream is = null;

		try {
			socket = new Socket(InetAddress.getByName(ip), port);
			os = new DataOutputStream(socket.getOutputStream());
			is = new DataInputStream(socket.getInputStream());
		} catch (UnknownHostException e) {
			mmib.sendMessage("{'error': true, 'message':'UNKNOWN HOST'}");
			System.err.println("Don't know about host: hostname");
		} catch (IOException e) {

			mmib.sendMessage("{'error': true, 'message':'COULD NOT GET I/O'}");
			System.err
					.println("Couldn't get I/O for the connection to: hostname");
		}

		if (socket != null && os != null && is != null) {
			try {

				os.writeBytes(s + "\r\n");
				
				mmib.sendMessage("{'error': false, 'message':'GW MESSAGE SEND'}");
				System.out.println("GW message send");
				
				String logString = "";
				char c;
				int i;
				
				 while((i=is.read())!=-1)
		         {
		            c=(char)i;
					
					if(c=='\n' || c == '\r')
						break;
					System.out.print(c);
					logString += c;
				}
				mmib.sendMessage("{'error': false, 'message':'received from GW: "+logString+"'}");
				os.close();
				is.close();
				socket.close();
				mmib.sendMessage("{'error': false, 'message':'Connection to GW closed.'}");

			} catch (UnknownHostException e) {
				mmib.sendMessage("{'error': true, 'message':'UNKNOWN HOST'}");
				System.err.println("Trying to connect to unknown host: " + e);
			} catch (IOException e) {
				mmib.sendMessage("{'error': true, 'message':'COULD NOT GET I/O'}");
				System.err.println("IOException:  " + e);
			}
		}
	}

	public class MyMessageInbound extends MessageInbound {
		WsOutbound myoutbound;

		private InetAddress bindAddress;
		private int port = 80;
		private Thread serverThread;

		private int gwServerPort;
		private String gwServerIP;


		private boolean authenticateUser(String un, String pw) {
			// todo get data from database
			if (un.equals("admin") && pw.equals("admin"))
				return true;
			else
				return false;
		}

		public void sendMessage(String msg) {
			//for (MyMessageInbound mmib : mmiList) {
			MyMessageInbound mmib = this;
				CharBuffer buffer = CharBuffer.wrap(msg);
				try {
					mmib.myoutbound.writeTextMessage(buffer);
					mmib.myoutbound.flush();
				} catch (IOException e) {
					e.printStackTrace();
				}

			//}
		}

		@Override
		public void onOpen(WsOutbound outbound) {
			try {
				System.out.println("Open Client.");
				this.myoutbound = outbound;
				mmiList.add(this);
				outbound.writeTextMessage(CharBuffer
						.wrap("{'error': false, 'message':'CONNECTION ACCEPTED'}"));
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		@Override
		public void onClose(int status) {
			System.out.println("Closing websocket client.");
			mmiList.remove(this);
		}

		@Override
		public void onTextMessage(CharBuffer cb) throws IOException {

			System.out.println("Accept Message : " + cb);
			MyMessageInbound mmib = this;
			//for (MyMessageInbound mmib : mmiList) {

				String json = cb.toString();

				Gson gson = new Gson();

				try {
					try {


						CommandContainer prod = gson.fromJson(json,
								CommandContainer.class);
						// Product prod = gson.fromJson(json, Product.class);

						CharBuffer buffer = CharBuffer
								.wrap("{'error': false, 'message':'ACCEPTED JSON, command: "
										+ prod.getCommand() + "'}");
						mmib.myoutbound.writeTextMessage(buffer);
						mmib.myoutbound.flush();

						if (prod.getCommand().equals("LOGIN")) {
							System.out.println("Logging in...");
							this.sendMessage("{'error':false, 'module':'debug', 'message':'New login attempt.'}");

							LoginData ld = prod.getLoginData();

							String username = ld.getUsername();
							String password = ld.getPassword();

							if (authenticateUser(username, password)) {
								this.sendMessage("{'error':false, 'module':'login', 'message':'LOGIN SUCCEEDED'}");
								//TODO: enable global logged in bool
							} else {
								this.sendMessage("{'error':true, 'module':'login', 'message':'LOGIN FAILED'}");
							}

						}
						
						else if (prod.getCommand().equals("SAVE_DATA")) {
							writeToXML(prod.getData(), mmib);
						}
						else if (prod.getCommand().equals("START_PA_SERVER")) { //TODO: Check if logged in
							if(this.serverThread != null){
								if(!this.serverThread.isAlive() && pacs == null){
									try {
										System.out.println("Starting PA server...");
										pacs = new PACommunicationServer(
												this.bindAddress, this.port, this);
										this.serverThread = new Thread(pacs);
										this.serverThread.start();

									} catch (Exception e) {
										this.sendMessage("{'error':true, 'message':'could not start server thread: "+e+"'}");
										e.printStackTrace();
									}
								}
								else 
									this.sendMessage("{'error':true, 'message':'PA communication server already running?'}");
							}
							else if(pacs == null){
								try {
									System.out.println("Starting PA server...");
									pacs = new PACommunicationServer(
											this.bindAddress, this.port, this);
									this.serverThread = new Thread(pacs);
									this.serverThread.start();

								} catch (Exception e) {
									this.sendMessage("{'error':true, 'message':'could not start server thread: "+e+"'}");
									
									e.printStackTrace();
								}
							}
							else
								this.sendMessage("{'error':true, 'message':'Server already started. Listening at: 0.0.0.0:" + pacs.getLocalPort()+" '}");
							
						}

						else if (prod.getCommand().equals("STOP_PA_SERVER")) {
							try {
								System.out.println("Stopping PA server...");

								buffer = CharBuffer
										.wrap("{'error': false, 'message':'Stopping PA Server'}");
								mmib.myoutbound.writeTextMessage(buffer);
								mmib.myoutbound.flush();

								pacs.terminate();
							} catch (Exception e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						} else if (prod.getCommand().equals("CREATE_PA")) {
							String serverData = prod.getData();
							String[] parts = serverData.split(":");

							if (parts.length == 2) {

								if (parts[0].equals("")
										|| parts[0].equals("undefined")) {
									this.gwServerIP = "localhost";
								} else {
									this.gwServerIP = parts[0];
								}
								if (parts[1].equals("")
										|| parts[1].equals("undefined")) {
									this.gwServerPort = Integer
											.parseInt("9090");
								} else {
									this.gwServerPort = Integer
											.parseInt(parts[1]);
								}
							} else {
								this.gwServerIP = "localhost";
								this.gwServerPort = 9090;
							}
							try {
								System.out.println("Starting PA...");

								buffer = CharBuffer
										.wrap("{'error': false, 'message':'Sending command string to "+ this.gwServerIP + " on port "
												+ this.gwServerPort + "...'}");
								mmib.myoutbound.writeTextMessage(buffer);
								mmib.myoutbound.flush();
								
								CharBuffer.wrap(cb.toString().replace("," + prod.getData(), "s"));
								
								sendToGWServer(cb, this.gwServerIP,
										this.gwServerPort, mmib);

							} catch (Exception e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}

					} catch (JsonSyntaxException jse) {
						// could not parse json
						System.out.println("could not parse json");

						CharBuffer buffer = CharBuffer
								.wrap("{'error': true, 'message':'NO VALID JSON'}");
						mmib.myoutbound.writeTextMessage(buffer);
						mmib.myoutbound.flush();

					}
				} finally {
				}
			//}
		}

		@Override
		public void onBinaryMessage(ByteBuffer bb) throws IOException {
			System.out.println("Binary exception");
		}

	}

	@Override
	protected StreamInbound createWebSocketInbound(String arg0,
			HttpServletRequest arg1) {

		return new MyMessageInbound();
	}
}
