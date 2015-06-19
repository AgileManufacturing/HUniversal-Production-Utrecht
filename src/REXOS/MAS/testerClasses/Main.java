package MAS.testerClasses;

import java.io.IOException;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.FileNotFoundException;

import org.json.JSONException;

public class Main {
    public static void main(String args[]) throws IOException, InterruptedException {
        TestSocketServer testsocketserver = new TestSocketServer(5000);

        testsocketserver.start();
        System.out.println("TestSocketServer started (" + testsocketserver.getAddress().getHostString() + ":" + testsocketserver.getPort() + ").");

        try {
            testsocketserver.loadJson("test.json");
            System.out.println("Successfully loaded json file");
        } catch (FileNotFoundException e) {
            System.out.println(e.getMessage());
        } catch (JSONException e) {
            System.out.println(e.getMessage());
        }

        InputStreamReader converter = new InputStreamReader(System.in);
        BufferedReader in = new BufferedReader(converter);

        // Loop forever (will break when finished, i.e. breaking out of the loop on user input)
        while (true) {
            System.out.print("> ");

            boolean finish = false;
            String  line   = in.readLine();

            String  cmd[]  = line.split(" ");

            // Evaluate input given
            switch (cmd[0]) {
                // Load a json file
                case "load":
                    // Accepts one argument, which is the json filename
                    if (cmd.length < 2 || cmd.length > 2) {
                        System.out.println("usage: reload <json filename>");
                        break;
                    }

                    try {
                        testsocketserver.loadJson(cmd[1]);
                        System.out.println("Successfully loaded json file");
                    } catch (FileNotFoundException e) {
                        System.out.println(e.getMessage());
                    } catch (JSONException e) {
                        System.out.println(e.getMessage());
                    }

                    break;
                // Terminate the program
                case "exit":
                case "quit":
                    finish = true;

                    break;
                // Don't act
                case "":
                    break;
                // Unknown command given
                default:
                    System.out.println("Unknown command '" + cmd[0] + "'");

                    break;
            }

            if (finish == true) {
                break;
            }
        }

        //
        System.out.println("Closing websocket server");
        testsocketserver.stop();
    }
}
