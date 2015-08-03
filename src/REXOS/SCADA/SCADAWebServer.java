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
    /**
     * Starting point of web server. Initializes HttpServer; binds to certain port.
     * Specifies handler that will handle all requests coming in.
     * @param  args      Arguments
     * @throws Exception When server is started already.
     */
    public static void main(String[] args) throws Exception {
        HttpServer server = HttpServer.create(new InetSocketAddress(8080), 0);
        server.createContext("/", new MyHandler());
        server.setExecutor(null);
        server.start();

        System.out.println("SCADA web server started");
    }

    /**
     * The Handler class that will be used to handle the requests coming in.
     */
    static class MyHandler implements HttpHandler {
        /**
         * THe handle method called whn a request comes in.
         * @param  exchange    HttpExchange containing headers and buffer that's to be filled.
         * @throws IOException When file buffer could not be created or filled.
         */
        public void handle(HttpExchange exchange) throws IOException {
        	URI    uri  = exchange.getRequestURI();
        	String path = "www" + uri.getPath();
        	// If there is a directory specified, add index.html as default file to serve
        	if (path.charAt(path.length() - 1) == '/') {
        		path += "index.html";
        	}

        	System.out.print(exchange.getRemoteAddress().getAddress().toString() + " ");

        	File file = new File(path);

            // Check if the file exists.
        	if (!file.isFile()) {
                // Default 404 content letting the user know stuff went wrong.
				String response = "404 - Not Found";
                // Set correct response headers and body.
				exchange.sendResponseHeaders(404, response.length());
				OutputStream os = exchange.getResponseBody();
				os.write(response.getBytes());
				os.close();

				System.out.println("File '" + path + "' not found - 404.");

        		return;
        	} else {
				System.out.println("File '" + path + "' sent.");
        	}

            // Figure out COntent-Type header from the file name.
            String content_type = Files.probeContentType(file.toPath());
            if (content_type == null) {
                content_type = "text/plain";
            }
            Headers hdrs = exchange.getResponseHeaders();
            hdrs.add("Content-Type", content_type);
            exchange.sendResponseHeaders(200, 0);

            // Create the stream that acts as the content body of the response.
            OutputStream    os     = exchange.getResponseBody();
            FileInputStream fs     = new FileInputStream(file);
            final byte[]    buffer = new byte[1024];
            int             count  = 0;
            while ((count = fs.read(buffer)) >= 0) {
            	os.write(buffer, 0, count);
            }
            fs.close();
            os.close();
        }
    }
}
