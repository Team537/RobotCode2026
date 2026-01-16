package frc.robot.network;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import com.google.gson.Gson;

/**
 * TimeSyncClient performs UDP time synchronization with a Raspberry Pi.
 *
 * This implementation:
 *  - Uses epoch-based nanoseconds (Instant.now()) for cross-device comparability (NOT System.nanoTime()).
 *  - Sends JSON requests containing a sequence number (seq) so replies can be matched and reordering is harmless.
 *  - Computes offset/delay using the standard four-timestamp exchange:
 *      offset = ((t2 - t1) + (t3 - t4)) / 2
 *      delay  = (t4 - t1) - (t3 - t2)
 *  - Improves robustness by selecting the best samples (lowest delay) before averaging offset.
 *  - Avoids silent encoding mistakes and adds defensive parsing/validation.
 *
 * Python server compatibility:
 *  - Works with the refined TimeSyncServer I provided:
 *      request: {"type":"TIME_SYNC","seq":<int>}
 *      response: {"t2":<int>,"t3":<int>,"seq":<int>}
 *
 * @author Cameron Myhre
 * @version 1.2.0
 */
public final class TimeSyncClient {

    private final String piIp;
    private final int piPort;
    private final int numSamples;
    private final int socketTimeoutMs;
    private final int interSampleSleepMs;
    private final int bestSampleCount; // how many of the lowest-delay samples to average
    private final Gson gson = new Gson();

    /** JSON request payload sent to the Pi. */
    private static final class TimeSyncRequest {
        @SuppressWarnings("unused")
        public final String type = "TIME_SYNC";
        public final int seq;

        private TimeSyncRequest(int seq) {
            this.seq = seq;
        }
    }

    /** JSON response payload received from the Pi. */
    public static final class TimeSyncResponse {
        public long t2; // Pi receive time (epoch ns)
        public long t3; // Pi send time (epoch ns)
        public Integer seq; // echoed sequence (optional, but expected)

        public boolean hasValidTimes() {
            return t2 > 0 && t3 > 0 && t3 >= t2;
        }
    }

    /** One measurement sample. */
    private static final class Sample {
        final long offsetNs;
        final long delayNs;
        final int seq;

        Sample(long offsetNs, long delayNs, int seq) {
            this.offsetNs = offsetNs;
            this.delayNs = delayNs;
            this.seq = seq;
        }
    }

    /**
     * Constructs a TimeSyncClient.
     *
     * @param piIp                 Pi IP address.
     * @param piPort               Pi UDP port.
     * @param numSamples           total samples to attempt.
     */
    public TimeSyncClient(String piIp, int piPort, int numSamples) {
        this(piIp, piPort, numSamples,
             1000,  // socketTimeoutMs
             50,    // interSampleSleepMs
             Math.max(3, Math.min(numSamples, Math.max(1, numSamples / 3)))); // bestSampleCount heuristic
    }

    /**
     * Advanced constructor for tuning.
     */
    public TimeSyncClient(
            String piIp,
            int piPort,
            int numSamples,
            int socketTimeoutMs,
            int interSampleSleepMs,
            int bestSampleCount
    ) {
        if (numSamples <= 0) throw new IllegalArgumentException("numSamples must be > 0");
        this.piIp = piIp;
        this.piPort = piPort;
        this.numSamples = numSamples;
        this.socketTimeoutMs = Math.max(50, socketTimeoutMs);
        this.interSampleSleepMs = Math.max(0, interSampleSleepMs);
        this.bestSampleCount = Math.max(1, Math.min(bestSampleCount, numSamples));
    }

    /**
     * Synchronize time with the Pi.
     *
     * @return double[]{avgOffsetNs, avgDelayNs, usedSamples, attemptedSamples}
     *         - avgOffsetNs: average clock offset (Pi - RIO) in ns using best samples
     *         - avgDelayNs:  average round-trip delay in ns using best samples
     *         - usedSamples: number of samples used in the "best" set
     *         - attemptedSamples: number of samples attempted
     */
    public double[] synchronizeTime() {
        List<Sample> samples = new ArrayList<>(numSamples);

        try (DatagramSocket socket = new DatagramSocket()) {
            socket.setSoTimeout(socketTimeoutMs);

            InetAddress piAddress = InetAddress.getByName(piIp);
            byte[] receiveBuffer = new byte[2048];

            for (int i = 0; i < numSamples; i++) {
                int seq = i;

                // --- T1: time request is "sent" (epoch ns) ---
                long t1 = epochNsNow();

                // Send JSON request with seq
                TimeSyncRequest req = new TimeSyncRequest(seq);
                byte[] sendData = gson.toJson(req).getBytes(StandardCharsets.UTF_8);
                DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, piAddress, piPort);
                socket.send(sendPacket);

                // Receive response
                DatagramPacket receivePacket = new DatagramPacket(receiveBuffer, receiveBuffer.length);
                try {
                    socket.receive(receivePacket);
                } catch (java.net.SocketTimeoutException timeout) {
                    // Skip this sample; continue
                    sleepQuiet(interSampleSleepMs);
                    continue;
                }

                // --- T4: time response is received (epoch ns) ---
                long t4 = epochNsNow();

                // Parse response JSON
                String jsonResponse = new String(
                        receivePacket.getData(),
                        receivePacket.getOffset(),
                        receivePacket.getLength(),
                        StandardCharsets.UTF_8
                );

                TimeSyncResponse resp;
                try {
                    resp = gson.fromJson(jsonResponse, TimeSyncResponse.class);
                } catch (Exception parseErr) {
                    // Malformed JSON; skip sample
                    sleepQuiet(interSampleSleepMs);
                    continue;
                }

                // Validate response
                if (resp == null || !resp.hasValidTimes()) {
                    sleepQuiet(interSampleSleepMs);
                    continue;
                }

                // If seq is present, ensure it matches the request we sent
                if (resp.seq != null && resp.seq != seq) {
                    // Out-of-order or stale response; ignore
                    sleepQuiet(interSampleSleepMs);
                    continue;
                }

                long t2 = resp.t2;
                long t3 = resp.t3;

                // Compute delay and offset
                // delay  = (t4 - t1) - (t3 - t2)
                // offset = ((t2 - t1) + (t3 - t4)) / 2
                long delay = (t4 - t1) - (t3 - t2);
                long offset = ((t2 - t1) + (t3 - t4)) / 2;

                // Basic sanity: delay should not be negative in normal conditions
                if (delay < 0) {
                    // Skip pathological sample
                    sleepQuiet(interSampleSleepMs);
                    continue;
                }

                samples.add(new Sample(offset, delay, seq));

                sleepQuiet(interSampleSleepMs);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        if (samples.isEmpty()) {
            // No valid samples
            return new double[]{0, 0, 0, numSamples};
        }

        // Sort by delay ascending (best = lowest delay)
        samples.sort(Comparator.comparingLong(s -> s.delayNs));

        int useN = Math.min(bestSampleCount, samples.size());
        List<Sample> best = samples.subList(0, useN);

        double avgOffset = best.stream().mapToLong(s -> s.offsetNs).average().orElse(0);
        double avgDelay = best.stream().mapToLong(s -> s.delayNs).average().orElse(0);

        return new double[]{avgOffset, avgDelay, useN, numSamples};
    }

    /**
     * Converts an Instant.now() to epoch nanoseconds, with overflow safety.
     */
    private static long epochNsNow() {
        Instant now = Instant.now();
        return Math.addExact(
                Math.multiplyExact(now.getEpochSecond(), 1_000_000_000L),
                now.getNano()
        );
    }

    private static void sleepQuiet(int ms) {
        if (ms <= 0) return;
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }
}