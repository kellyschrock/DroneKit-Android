package org.droidplanner.services.android.impl.core.MAVLink.connection;

import com.MAVLink.MAVLinkPacket;

import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus;

/**
 * Provides updates about the mavlink connection.
 */
public interface MavLinkConnectionListener {
    /**
     * Called when data is received via the mavlink connection.
     *
     * @param packet received data
     */
    void onReceivePacket(MAVLinkPacket packet);

    /**
     * Provides information about communication error.
     *
     * @param connectionStatus error information
     */
    void onConnectionStatus(LinkConnectionStatus connectionStatus);


    /**
     * Provides information about mavlink stats
     */
    void onMavlinkStatsUpdate(int receivedCount, int crcErrorCount, int lostPacketCount);


    /**
     * Called when data taken from the outgoing queue is sent
     * @param data array of encoded bytes created from the Mavlink Message which was added to the queue
     */
    void onBytesSent(byte[] data);

    /**
     * Called when an incoming mavlink message is parsed from the queue
     * @param numBytes the number of bytes in the message
     */
    void onReceivedBytesParsed(int numBytes);

    /**
     * Called when a outgoing mavlink messages is offered to the queue
     * @param packetData array of encoded bytes created from the Mavlink Message which was added to the queue
     */
    void onMessageQueued(byte[] packetData);


}
