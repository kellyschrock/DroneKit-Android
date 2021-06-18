package org.droidplanner.services.android.impl.utils;


import android.os.Handler;

import java.util.HashSet;
import java.util.Set;

/** Class for collecting/reporting stats on a mavlink connection */
public class LinkStats {
    public static class Stats {
        public final long when;
        public final long period;
        public final int messagesSent;
        public final int messagesReceived;
        public final long bytesSent;
        public final long bytesReceived;

        Stats(long when, long period, int msgSent, int msgReceived, long bytesSent, long bytesReceived) {
            this.when = when;
            this.period = period;
            this.messagesSent = msgSent;
            this.messagesReceived = msgReceived;
            this.bytesSent = bytesSent;
            this.bytesReceived = bytesReceived;
        }
    }

    public interface LinkRateListener {
        void onStatsUpdate(Stats stats);
    }

    private final static int PING_INTERVAL = 1000; // ms

    private static LinkStats sInstance;

    private final Set<LinkRateListener> listeners = new HashSet<>();
//    private final Handler handler = new android.os.Handler();

    private final Runnable heartbeat = new Runnable() {
        public void run() {
            if(doHeartbeat()) {
//                handler.postDelayed(this, PING_INTERVAL);
            }
        }
    };

    private int messageSentCount = 0;
    private int messageReceivedCount = 0;
    private int bytesSentCount = 0;
    private int bytesReceivedCount = 0;

    private boolean active = false;

    private LinkStats() { }

    public static LinkStats get() {
        if(sInstance == null)
            sInstance = new LinkStats();
        return sInstance;
    }

    public void onMessageSent() {
        messageSentCount++;
    }

    public void onMessageReceived() {
        messageReceivedCount++;
    }

    public void onBytesSent(int numSent) {
        bytesSentCount += numSent;
    }

    public void onBytesReceived(int numReceived) {
        bytesReceivedCount += numReceived;
    }

    public int getMessageSentCount() {
        return messageSentCount;
    }

    public int getMessageReceivedCount() {
        return messageReceivedCount;
    }

    public Stats getCurrentStats() {
        return new Stats(System.currentTimeMillis(), PING_INTERVAL, messageSentCount, messageReceivedCount, bytesSentCount, bytesReceivedCount);
    }

    public boolean addListener(LinkRateListener linkSpeedListener) {
        boolean added = listeners.add(linkSpeedListener);

        if (!active) {
            startHeartbeat();
        }

        return added;
    }

    public boolean removeListener(LinkRateListener linkSpeedListener) {
        boolean removed = listeners.remove(linkSpeedListener);

        if(listeners.isEmpty()) {
            stopHeartbeat();
        }

        return removed;
    }

    /** Resets tracked statistics to defaults */
    public void reset() {
        messageSentCount = 0;
        messageReceivedCount = 0;
        bytesReceivedCount = 0;
        bytesSentCount = 0;
        listeners.clear();
        stopHeartbeat();
    }

    private void startHeartbeat() {
        active = true;
//        handler.postDelayed(heartbeat, PING_INTERVAL);
    }

    private void stopHeartbeat() {
        active = false;
//        handler.removeCallbacks(heartbeat);
    }

    private boolean doHeartbeat() {
        final long now = System.currentTimeMillis();
        // Reset internal counters
        final int bytesSent = LinkStats.this.bytesSentCount;
        LinkStats.this.bytesSentCount = 0;

        final int bytesReceived = LinkStats.this.bytesReceivedCount;
        LinkStats.this.bytesReceivedCount = 0;

        final Stats stats = new Stats(now, PING_INTERVAL, messageSentCount, messageReceivedCount, bytesSent, bytesReceived);

        for (LinkRateListener listener : listeners) {
            listener.onStatsUpdate(stats);
        }

        return (active && !listeners.isEmpty());
    }
}
