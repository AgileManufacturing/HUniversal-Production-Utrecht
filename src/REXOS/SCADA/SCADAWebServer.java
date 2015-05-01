package SCADA;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;

import java.net.URI;
import java.io.FileInputStream;
import java.io.File;
import java.nio.file.Files;

import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

public class SCADAWebServer {
    public static void main(String[] args) throws Exception {
        HttpServer server = HttpServer.create(new InetSocketAddress(8080), 0);
        server.createContext("/", new MyHandler());
        server.setExecutor(null); // creates a default executor
        server.start();
    }

    static class MyHandler implements HttpHandler {
        public void handle(HttpExchange exchange) throws IOException {
        	URI    uri  = exchange.getRequestURI();
        	String path = "www/" + uri.getPath();
        	// If there is a directory specified, add index.html as default file to serve
        	if (path.charAt(path.length() - 1) == '/') {
        		path += "index.html";
        	}
        	File file = new File(path);

        	if (!file.isFile()) {
				String response = "404 - Not Found";
				exchange.sendResponseHeaders(404, response.length());
				OutputStream os = exchange.getResponseBody();
				os.write(response.getBytes());
				os.close();
        		return;
        	}
        	
        	Headers hdrs = exchange.getResponseHeaders();
        	hdrs.add("Content-Type", Files.probeContentType(file.toPath()));
        	exchange.sendResponseHeaders(200, 0);

            OutputStream    os     = exchange.getResponseBody();
            FileInputStream fs     = new FileInputStream(file);
            final byte[]    buffer = new byte[0x10000];
            int             count  = 0;
            while ((count = fs.read(buffer)) >= 0) {
            	os.write(buffer, 0, count);
            }
            fs.close();
            os.close();
        }
    }
}
