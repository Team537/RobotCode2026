package frc.robot.network;

import java.io.IOException;
import java.net.*;
import java.nio.charset.StandardCharsets;

import com.google.gson.Gson;

/**
 * UDPSender sends a single JSON object per UDP packet to a remote device
 * (e.g., Raspberry Pi / Jetson).
 *
 * Notes:
 *  - UDP is connectionless: there is no "connected" state.
 *  - Each send(...) is one datagram containing one JSON message.
 */
public final class UDPSender implements AutoCloseable {

    private final Gson gson = new Gson();
    private final Object lock = new Object();

    private DatagramSocket socket;
    private InetAddress targetAddress;
    private int targetPort;

    // Keep packets comfortably under typical MTU to reduce fragmentation.
    private int maxPayloadBytes = 1200;

    /**
     * Creates a new UDPSender that targets the given IP and port.
     */
    public UDPSender(String targetIp, int targetPort) throws IOException {
        setTarget(targetIp, targetPort);
        openSocket();
    }

    private void openSocket() throws SocketException {
        DatagramSocket socket = new DatagramSocket(); // ephemeral local port
        socket.setReuseAddress(true);
        synchronized (lock) {
            this.socket = socket;
        }
    }

    /**
     * Sets the remote destination for outgoing packets.
     */
    public void setTarget(String targetIp, int targetPort) throws UnknownHostException {
        InetAddress addr = InetAddress.getByName(targetIp);
        synchronized (lock) {
            this.targetAddress = addr;
            this.targetPort = targetPort;
        }
    }

    /**
     * Optional: adjust max payload size (bytes). Keep < ~1400 to avoid fragmentation.
     */
    public void setMaxPayloadBytes(int maxPayloadBytes) {
        synchronized (lock) {
            this.maxPayloadBytes = Math.max(256, maxPayloadBytes);
        }
    }

    /**
     * Sends an arbitrary object as JSON in a single UDP datagram.
     * The receiver should parse the datagram as a complete JSON message.
     *
     * @param message Any object serializable by Gson.
     */
    public void send(Object message) throws IOException {
        byte[] payload;
        InetAddress addr;
        int port;
        int maxBytes;
        DatagramSocket s;

        synchronized (lock) {
            ensureOpen();
            s = this.socket;
            addr = this.targetAddress;
            port = this.targetPort;
            maxBytes = this.maxPayloadBytes;
        }

        String json = gson.toJson(message);
        payload = json.getBytes(StandardCharsets.UTF_8);

        if (payload.length > maxBytes) {
            throw new IOException("UDP payload too large (" + payload.length + " bytes). "
                    + "Increase maxPayloadBytes or reduce message size.");
        }

        DatagramPacket packet = new DatagramPacket(payload, payload.length, addr, port);
        s.send(packet);
    }

    /**
     * 
     */
    private void ensureOpen() {
        if (socket == null || socket.isClosed()) {
            throw new IllegalStateException("UDP socket is not open.");
        }
        if (targetAddress == null) {
            throw new IllegalStateException("Target address not set.");
        }
        if (targetPort <= 0) {
            throw new IllegalStateException("Target port not set.");
        }
    }

    /**
     * UDP doesn't have a real connected state; this just indicates if the local socket exists.
     */
    public boolean isOpen() {
        synchronized (lock) {
            return socket != null && !socket.isClosed();
        }
    }

    @Override
    public void close() {
        synchronized (lock) {
            if (socket != null) {
                socket.close();
            }
            socket = null;
        }
    }
}
