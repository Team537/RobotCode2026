package frc.robot.network;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

import com.google.gson.Gson;

/**
 * TCPSender is responsible for establishing a TCP connection to a remote device
 * (e.g., Raspberry Pi)
 * and sending configuration data in JSON format over the network.
 * 
 * @author Cameron Myhre
 * @since v2.0.0
 */
public class TCPSender implements AutoCloseable {

    private Socket socket;
    private BufferedWriter writer;
    private final Gson gson = new Gson(); // Gson instance for JSON serialization
    private final Object lock = new Object(); // Lock for thread-safe operations

    /**
     * Creates a new TCPSender object and establishes a TCP connection to the
     * specified Raspberry Pi.
     *
     * @param piIp   The IP address of the Raspberry Pi.
     * @param piPort The port number on which the Raspberry Pi is listening.
     * @throws IOException if an I/O error occurs when creating the socket or output
     *                     stream.
     */
    public TCPSender(String piIp, int piPort) throws IOException {
        connect(piIp, piPort);
    }

    /**
     * Establishes a TCP connection to the specified IP and port.
     * 
     * @param piIp The IP address of the Raspberry Pi.
     * @param piPort The port number on which the Raspberry Pi is listening.
     * @throws IOException if an I/O error occurs when creating the socket or output stream.
     */
    private void connect(String piIp, int piPort) throws IOException{
        
        Socket socket = new Socket();
        socket.setTcpNoDelay(true);
        socket.setKeepAlive(true);
        socket.connect(new InetSocketAddress(piIp, piPort), 5000);

        BufferedWriter writer = new BufferedWriter(
            new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8)
        );

        synchronized (lock) {
            this.socket = socket;
            this.writer = writer;
        }
    }
    
    /**
     * Sends a configuration object to the connected Raspberry Pi.
     * The object is serialized to JSON before transmission.
     *
     * @param config The configuration object to send.
     * @throws IOException 
     * @throws IllegalStateException if the TCP connection is not open.
     */
    public void sendConfiguration(Object config) throws IOException {

        // Convert the configuration object to JSON.
        String json = gson.toJson(config) + "\n";
        synchronized (lock) {
            ensureConnected();
            writer.write(json);
            writer.flush();
        }
    }

    /**
     * Ensures that the TCP connection is open.
     */
    private void ensureConnected() {
        if (!isConnected()) throw new IllegalStateException("TCP connection not open.");
    }

    /**
     * Closes the TCP connection and releases all associated resources.
     */
    @Override
    public void close() {
        synchronized (lock) {
            try { if (writer != null) writer.close(); } catch (IOException ignored) {}
            try { if (socket != null) socket.close(); } catch (IOException ignored) {}
            writer = null;
            socket = null;
        }
    }

    /**
     * Attempts to reconnect to the specified IP/port. This method will close any
     * existing
     * connection, then create and connect a new socket, and recreate the
     * PrintWriter.
     *
     * @param piIp   The IP address of the Raspberry Pi.
     * @param piPort The port number on which the Raspberry Pi is listening.
     * @throws IOException if an I/O error occurs when creating the socket or output
     *                     stream.
     */
    public void reconnect(String piIp, int piPort) throws IOException {
        synchronized (lock) {
            close();
        }
        connect(piIp, piPort);
    }

    /**
     * Checks if the TCP connection is currently open.
     *
     * @return true if the connection is open, false otherwise.
     */
    public boolean isConnected() {
        synchronized (lock) {
            return socket != null && socket.isConnected() && !socket.isClosed() && !socket.isOutputShutdown();
        }
    }
}
