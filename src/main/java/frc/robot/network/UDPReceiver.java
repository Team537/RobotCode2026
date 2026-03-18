package frc.robot.network;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

import com.google.gson.Gson;

/**
 * 
 * filed23.setLists, then put on dashboard.
 * UDPReceiver is responsible for receiving JSON data from a Raspberry Pi over UDP.
 * This class runs a separate thread to continuously listen for incoming data without blocking the main robot control loop.
 * <hr>
 * @author Cameron Myhre
 * @version 1.1.0
 * @since 0.1.0-26
 */
public final class UDPReceiver<T> implements AutoCloseable {
    private final Gson gson = new Gson();
    private final Class<T> classReceiveType;

    private final int port;
    private final AtomicReference<T> latest = new AtomicReference<>();

    private volatile boolean running = false;   
    private Thread thread;
    private DatagramSocket socket;

    private int lastPacketNumber = -1; // uninitialized

    /**
     * Constructs a UDPReceiver that listens on the specified port for JSON data of type T.
     * 
     * @param port The UDP port to listen on.
     * @param classReceiveType The class type of the data being received.
     */
    public UDPReceiver(int port, Class<T> classReceiveType) {
        this.port = port;
        this.classReceiveType = classReceiveType;
    }

    /**
     * Starts the UDP receiver thread.
     */
    public void start() {
        if (running) return;
        running = true;

        // Start the receiver thread
        thread = new Thread(() -> {

            // Buffer for incoming packets
            byte[] buffer = new byte[4096];

            // Address binding and receiving loop
            try (DatagramSocket ds = new DatagramSocket(port)) {
                socket = ds;

                // Receiving loop
                while (running) {

                    // Receive packet
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    ds.receive(packet);
                    
                    // Parse JSON
                    String json = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
                    T parsed = gson.fromJson(json, classReceiveType);
                    latest.set(parsed);
                }
            } catch (Exception e) {
                if (running) e.printStackTrace();
            } finally {
                running = false;
            }
        }, "UDPReceiver-" + port);

        thread.setDaemon(true);
        thread.start();
    }

    /**
     * Gets the latest received data.
     * 
     * @return The latest data of type T received over UDP.
     */
    public T getLatest() {
        return latest.get();
    }

    /**
     * Closes the UDP receiver and releases all associated resources.
     */
    @Override
    public void close() {
        running = false;
        DatagramSocket ds = socket;
        if (ds != null) ds.close(); // unblocks receive()
    }
}
