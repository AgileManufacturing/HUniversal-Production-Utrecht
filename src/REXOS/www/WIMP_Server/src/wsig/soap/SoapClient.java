/*****************************************************************
JADE - Java Agent DEvelopment Framework is a framework to develop 
multi-agent systems in compliance with the FIPA specifications.
Copyright (C) 2002 TILAB

GNU Lesser General Public License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, 
version 2.1 of the License. 

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*****************************************************************/

package wsig.soap;

import java.io.*;
import java.net.*;

public class SoapClient {

	public static String sendFileMessage(String SOAPUrl, String fileName) {

		// Read file
		ByteArrayOutputStream bout = null;
		try {
			FileInputStream fin = new FileInputStream(fileName);
			bout = new ByteArrayOutputStream();
			copy(fin,bout);
			fin.close();
		} catch(Exception e) {
			String resp = "Error reading file";
			System.out.println(resp);
			e.printStackTrace();
			return resp; 
		}

		// Convert in byte array
		byte[] byteMessage = bout.toByteArray();
		System.out.println("SOAP request:");
		System.out.println(new String(byteMessage));

		// Send message
		return sendMessage(SOAPUrl, byteMessage);
	}

	public static String sendStringMessage(String SOAPUrl, String SOAPmessage) {

		// Convert in byte array
		byte[] byteMessage = SOAPmessage.getBytes();

		// Send message
		return sendMessage(SOAPUrl, byteMessage);
	}

	private static String sendMessage(String SOAPUrl, byte[] byteMessage) {

		String resp = null;
		boolean requestSent = false;
		HttpURLConnection httpConn = null;
		try {
			// Create the connection
			URL url = new URL(SOAPUrl);
			URLConnection connection = url.openConnection();
			httpConn = (HttpURLConnection) connection;

			// Set the appropriate HTTP parameters.
			httpConn.setRequestProperty( "Content-Length", String.valueOf( byteMessage.length ) );
			httpConn.setRequestProperty("Content-Type", "text/xml; charset=utf-8");
			httpConn.setRequestProperty("SOAPAction", "");
			httpConn.setRequestMethod( "POST" );
			httpConn.setDoOutput(true);
			httpConn.setDoInput(true);

			// Send the soap message
			OutputStream out = httpConn.getOutputStream();
			out.write(byteMessage);    
			out.close();

			requestSent = true;
			
			// Check response code
			if (httpConn.getResponseCode() == HttpURLConnection.HTTP_OK) {
				
				// Read the response
				InputStreamReader isr =	new InputStreamReader(httpConn.getInputStream());
				BufferedReader in = new BufferedReader(isr);

				String inputLine;
				StringBuffer sb = new StringBuffer();
				while ((inputLine = in.readLine()) != null) {
					sb.append(inputLine);
				}

				in.close();

				resp = sb.toString();
			} else {

				// Get error message
				resp = httpConn.getResponseMessage();
			}
		} catch(Exception e) {
			resp = (requestSent ? "Error response received" : "Error sending soap message")+" - "+e.getMessage();
			System.out.println(resp);
			e.printStackTrace();
		} finally {
			try { httpConn.disconnect(); } catch(Exception e) {}
		}

		return resp;
	}

	private static void copy(InputStream in, OutputStream out) throws IOException {
		synchronized (in) {
			synchronized (out) {
				byte[] buffer = new byte[256];
				while (true) {
					int bytesRead = in.read(buffer);
					if (bytesRead == -1) break;
					out.write(buffer, 0, bytesRead);
				}
			}
		}
	} 

	public static void main(String[] args) throws Exception {
		
		System.out.println("SOAP Client to inwoke a webservice");
		System.out.println("");
		
		if (args == null || args.length < 2) {
			System.out.println("usage: SoapClient URL FILE");
			System.out.println("where:");
			System.out.println("URL: soap web-server url");
			System.out.println("FILE: xml file with soap message request");
			return;
		}
		
		String SOAPUrl = args[0];
		String SOAPFile =args[1]; 
		System.out.println("Web-service: "+SOAPUrl);
		String resp = SoapClient.sendFileMessage(SOAPUrl, SOAPFile);

		System.out.println("");
		System.out.println("SAOP response:");
		System.out.println(resp);
	}
}
