package com.o3dr.services.android.lib.drone.connection

import com.o3dr.services.android.lib.drone.connection.ConnectionType
import android.support.annotation.IntDef
import java.lang.annotation.Retention
import java.lang.annotation.RetentionPolicy

/**
 * Contains constants used for the connection parameters.
 */
object ConnectionType {
    /**
     * USB connection type
     */
    const val TYPE_USB = 0

    /**
     * Key used to retrieve the usb baud rate from the connection parameter bundle.
     */
    const val EXTRA_USB_BAUD_RATE = "extra_usb_baud_rate"

    /**
     * Default value for the usb baud rate.
     */
    const val DEFAULT_USB_BAUD_RATE = 57600

    /**
     * UDP connection type
     */
    const val TYPE_UDP = 1

    /**
     * Key used to retrieve the udp server port from the connection parameter bundle
     */
    const val EXTRA_UDP_SERVER_PORT = "extra_udp_server_port"

    /** Key for UDP server IP  */
    const val EXTRA_UDP_SERVER_IP = "extra_udp_server_ip"

    /**
     * Default value for the upd server port.
     */
    const val DEFAULT_UDP_SERVER_PORT = 14550

    /** Default value for UDP server IP  */
    const val DEFAULT_UDP_SERVER_IP = "10.1.1.10"

    /**
     * Key used to retrieve the ip address of the udp server to ping.
     */
    const val EXTRA_UDP_PING_RECEIVER_IP = "extra_udp_ping_receiver_ip"

    /**
     * Key used to retrieve the port of the udp server to ping.
     */
    const val EXTRA_UDP_PING_RECEIVER_PORT = "extra_udp_ping_receiver_port"

    /**
     * Ping payload.
     */
    const val EXTRA_UDP_PING_PAYLOAD = "extra_udp_ping_payload"

    /**
     * How often should the udp ping be performed.
     */
    const val EXTRA_UDP_PING_PERIOD = "extra_udp_ping_period"
    const val DEFAULT_UDP_PING_PERIOD = 10000L //10 seconds

    /**
     * TCP connection type
     */
    const val TYPE_TCP = 2

    /**
     * Key used to retrieve the tcp server ip from the connection parameter bundle
     */
    const val EXTRA_TCP_SERVER_IP = "extra_tcp_server_ip"

    /**
     * Key used to retrieve the tcp server port from the connection parameter bundle
     */
    const val EXTRA_TCP_SERVER_PORT = "extra_tcp_server_port"

    /**
     * Default value for the tcp server port.
     */
    const val DEFAULT_TCP_SERVER_PORT = 5763

    /**
     * Bluetooth connection type
     */
    const val TYPE_BLUETOOTH = 3

    /**
     * Key used to retrieve the bluetooth address from the connection parameter bundle
     */
    const val EXTRA_BLUETOOTH_ADDRESS = "extra_bluetooth_address"

    /**
     * Solo vehicle connection type
     * Opens a UDP connection at port 14550 once on the correct wifi network.
     */
    const val TYPE_SOLO = 101

    /**
     * Used to retrieve the id (wifi ssid) of the solo vehicle link.
     */
    const val EXTRA_SOLO_LINK_ID = "extra_solo_link_id"

    /** Custom connection type
     * Uses a custom MavlinkConnectionManager provided by the client
     */
    const val TYPE_CUSTOM = 99
    const val EXTRA_CUSTOM_CONNECTION_ID = "extra_custom_connection_id"

    /**
     * Used to retrieve the password to access the solo wifi network.
     */
    const val EXTRA_SOLO_LINK_PASSWORD = "extra_solo_link_password"
    fun getConnectionTypeLabel(@Type connectionType: Int): String? {
        return when (connectionType) {
            TYPE_BLUETOOTH -> "bluetooth"
            TYPE_TCP -> "tcp"
            TYPE_UDP -> "udp"
            TYPE_USB -> "usb"
            TYPE_SOLO -> "solo"
            TYPE_CUSTOM -> "custom"
            else -> null
        }
    }

    @IntDef(TYPE_USB.toLong(), TYPE_UDP.toLong(), TYPE_TCP.toLong(), TYPE_BLUETOOTH.toLong(), TYPE_SOLO.toLong(), TYPE_CUSTOM.toLong())
    @Retention(RetentionPolicy.SOURCE)
    annotation class Type
}
