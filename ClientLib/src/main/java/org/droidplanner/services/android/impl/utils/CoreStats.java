package org.droidplanner.services.android.impl.utils;


import java.util.ArrayList;

public class CoreStats {

    private CoreStats() { }

    private static CoreStats mInstance;

    private final static int DATA_RATE_PERIOD = 1000; // 1 second

    private int numMessagesSent = 0;
    private int numMessagesReceived = 0;
    private int numBytesSent = 0;
    private int numBytesReceived = 0;

    private ArrayList<DataLinkRateListener> dataRateListeners = new ArrayList<>();

    public interface DataLinkRateListener {
        public void receivedRate(long timestamp, int numBytesSent, int numBytesReceived, int period);
    }

    StoppableThread dataRateUpdateThread;



    public static CoreStats get() {
        if(mInstance == null)
            mInstance = new CoreStats();
        return mInstance;
    }

    public void notifyMessageSent() {
        numMessagesSent++;
    }

    public int getNumMessagesSent() {
        return numMessagesSent;
    }

    public void notifyMessageReceived() {
        numMessagesReceived++;
    }

    public int getNumMessagesReceived() {
        return numMessagesReceived;
    }

    public void notifySentBytes(int numSent) {
        numBytesSent += numSent;
    }

    public void notifyReceivedBytes(int numReceived) {
        numBytesReceived += numReceived;
    }

    public boolean registerDataRateListener(DataLinkRateListener linkSpeedListener) {
        boolean added = dataRateListeners.add(linkSpeedListener);

        if (dataRateUpdateThread == null) {
            dataRateUpdateThread = new StoppableThread() {
                @Override
                public void run() {
                    try {
                        while (keepGoing) {
                            long timeStamp = System.currentTimeMillis();
                            int numBytesSent = CoreStats.get().numBytesSent;
                            CoreStats.get().numBytesSent = 0;

                            int numBytesReceived = CoreStats.get().numBytesReceived;
                            CoreStats.get().numBytesReceived = 0;

                            for (DataLinkRateListener listener : dataRateListeners) {
                                listener.receivedRate(timeStamp, numBytesSent, numBytesReceived, DATA_RATE_PERIOD);
                            }
                            Thread.sleep(DATA_RATE_PERIOD);
                        }
                    } catch (Throwable e) {
                        System.out.println();
                        e.printStackTrace();
                    }
                }
            };
            dataRateUpdateThread.start();
        }

        return added;
    }

    public boolean unregisterDataRateListener(DataLinkRateListener linkSpeedListener) {
        boolean removed = false;

        int index = dataRateListeners.indexOf(linkSpeedListener);
        if(index != -1)
            removed = dataRateListeners.remove(index) != null;

        if(dataRateListeners.isEmpty() && dataRateUpdateThread != null) {
            dataRateUpdateThread.stopMe();
            dataRateUpdateThread = null;
        }
        return removed;
    }

    /**
     * Resets tracked statistics to defaults
     */
    public void reset() {
        numMessagesSent = 0;
        numMessagesReceived = 0;
        dataRateListeners = new ArrayList<>();
    }

    static class StoppableThread extends Thread {
        // Don't tell me what I can and cant stop
        boolean keepGoing = true;
        public void stopMe() {
            keepGoing = false;
        }
    }
}
