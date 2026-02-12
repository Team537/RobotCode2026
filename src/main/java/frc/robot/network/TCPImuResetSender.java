package frc.robot.network;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

import com.google.gson.Gson;

/**
 * Sends IMU reset commands over TCP as newline-delimited JSON.
 *
 * <p>Each reset command is emitted as a single NDJSON line:
 * <pre>
 * {"cmd":"zero_imu","cmd_id":12345}
 * </pre>
 *
 * <p>This class is thread-safe for calls to {@link #sendZeroImu(long)} and {@link #sendZeroImuBurst(long, int)}.
 */
public final class TCPImuResetSender implements AutoCloseable {

    /**
     * Command payload for IMU zeroing.
     */
    private static final class ZeroImuCommand {
        @SuppressWarnings("unused")
        final String cmd = "zero_imu";

        @SuppressWarnings("unused")
        final long cmd_id;

        /**
         * Creates a zero IMU command with a caller-provided identifier.
         *
         * @param cmdId command identifier used for deduplication on the receiver
         */
        ZeroImuCommand(long cmdId) {
            this.cmd_id = cmdId;
        }
    }

    private final Gson gson = new Gson();
    private final Object lock = new Object();

    private final String host;
    private final int port;

    private Socket socket;
    private BufferedWriter writer;

    private int connectTimeoutMs = 250;

    /**
     * Creates a sender and connects immediately to the specified host and port.
     *
     * @param host remote host or IP address
     * @param port remote TCP port
     * @throws IOException if the connection cannot be established
     */
    public TCPImuResetSender(String host, int port) throws IOException {
        this.host = host;
        this.port = port;
        connect();
    }

    /**
     * Sets the TCP connect timeout used by subsequent connects or reconnects.
     *
     * @param connectTimeoutMs timeout in milliseconds
     */
    public void setConnectTimeoutMs(int connectTimeoutMs) {
        this.connectTimeoutMs = Math.max(1, connectTimeoutMs);
    }

    /**
     * Establishes a TCP connection and prepares the writer for NDJSON output.
     *
     * @throws IOException if the socket cannot be opened or connected
     */
    private void connect() throws IOException {
        Socket socket = new Socket();
        socket.setTcpNoDelay(true);
        socket.setKeepAlive(true);
        socket.connect(new InetSocketAddress(host, port), connectTimeoutMs);

        BufferedWriter writer = new BufferedWriter(
                new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8)
        );

        synchronized (lock) {
            this.socket = socket;
            this.writer = writer;
        }
    }

    /**
     * Sends a single IMU reset command.
     *
     * @param cmdId command identifier used for deduplication on the receiver
     * @throws IOException if an I/O error occurs while writing to the socket
     * @throws IllegalStateException if the sender is not currently connected
     */
    public void sendZeroImu(long cmdId) throws IOException {
        String jsonLine = gson.toJson(new ZeroImuCommand(cmdId)) + "\n";
        synchronized (lock) {
            ensureConnected();
            writer.write(jsonLine);
            writer.flush();
        }
    }

    /**
     * Sends the same IMU reset command multiple times on the same TCP connection.
     *
     * @param cmdId command identifier used for deduplication on the receiver
     * @param count number of times to send the command, clamped to at least 1
     * @throws IOException if an I/O error occurs while writing to the socket
     * @throws IllegalStateException if the sender is not currently connected
     */
    public void sendZeroImuBurst(long cmdId, int count) throws IOException {
        int n = Math.max(1, count);
        String jsonLine = gson.toJson(new ZeroImuCommand(cmdId)) + "\n";

        synchronized (lock) {
            ensureConnected();
            for (int i = 0; i < n; i++) {
                writer.write(jsonLine);
            }
            writer.flush();
        }
    }

    /**
     * Reconnects by closing any existing socket and establishing a new connection.
     *
     * @throws IOException if the new connection cannot be established
     */
    public void reconnect() throws IOException {
        synchronized (lock) {
            close();
        }
        connect();
    }

    /**
     * Determines whether the underlying socket is connected and usable for output.
     *
     * @return true if connected, false otherwise
     */
    public boolean isConnected() {
        synchronized (lock) {
            return socket != null
                    && socket.isConnected()
                    && !socket.isClosed()
                    && !socket.isOutputShutdown();
        }
    }

    /**
     * Ensures the sender is connected before performing socket writes.
     *
     * @throws IllegalStateException if the sender is not connected
     */
    private void ensureConnected() {
        if (!isConnected()) {
            throw new IllegalStateException("TCP IMU reset connection is not open.");
        }
    }

    /**
     * Closes the writer and socket and releases associated resources.
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
}
