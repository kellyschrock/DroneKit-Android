package org.droidplanner.services.android.impl.communication.service

import android.content.Context
import android.net.Uri
import android.text.TextUtils
import com.MAVLink.MAVLinkPacket
import com.MAVLink.Messages.MAVLinkMessage
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.drone.connection.ConnectionType
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.communication.connection.*
import org.droidplanner.services.android.impl.communication.connection.usb.UsbConnection
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkListener
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkProvider
import org.droidplanner.services.android.impl.core.MAVLink.connection.MavLinkConnection
import org.droidplanner.services.android.impl.core.MAVLink.connection.MavLinkConnectionListener
import org.droidplanner.services.android.impl.core.drone.manager.DroneCommandTracker
import org.droidplanner.services.android.impl.utils.connection.WifiConnectionHandler
import timber.log.Timber
import java.io.File
import java.net.InetAddress
import java.net.UnknownHostException

/**
 * Provide a common class for some ease of use functionality
 */
open class MAVLinkClient(private val context: Context, private val listener: DataLinkListener<MAVLinkPacket?>,
                         connParams: ConnectionParameter?, commandTracker: DroneCommandTracker?)
    : DataLinkProvider<MAVLinkMessage> {

    private val mConnectionListener: MavLinkConnectionListener = object : MavLinkConnectionListener {
        override fun onReceivePacket(packet: MAVLinkPacket) {
            listener.notifyReceivedData(packet)
        }

        override fun onConnectionStatus(connectionStatus: LinkConnectionStatus) {
            Timber.i("onConnectionStatus(): status=%s", connectionStatus)
            listener.onConnectionStatus(connectionStatus)
            when (connectionStatus.statusCode) {
                LinkConnectionStatus.DISCONNECTED -> closeConnection()
            }
        }

        override fun onMavlinkStatsUpdate(received: Int, crcErrors: Int, lostPackets: Int) {
            receivedCount = received
            crcErrorCount = crcErrors
            lostPacketCount = lostPackets
        }
    }
    private var mavlinkConn: AndroidMavLinkConnection? = null
    private var packetSeqNumber = 0
    private val connParams: ConnectionParameter
    private val commandTracker: DroneCommandTracker?
    override var receivedCount = 0
    override var crcErrorCount = 0
    override var lostPacketCount = 0
    private val connectionStatus: Int
        private get() = if (mavlinkConn == null) MavLinkConnection.MAVLINK_DISCONNECTED else mavlinkConn!!.connectionStatus

    /**
     * Setup a MAVLink connection based on the connection parameters.
     */
    @Synchronized
    override fun openConnection() {
        Timber.i("openConnection()")
        if (isConnected || isConnecting) {
            Timber.d("isConnected() || isConnecting()")
            return
        }
        val tag = toString()
        Timber.d("tag=%s mavlinkConn=%s", tag, mavlinkConn)

        //Create the mavlink connection
        val connectionType = connParams.connectionType
        val paramsBundle = connParams.paramsBundle
        if (mavlinkConn == null) {
            when (connectionType) {
                ConnectionType.TYPE_USB -> {
                    val baudRate = paramsBundle!!.getInt(ConnectionType.EXTRA_USB_BAUD_RATE,
                            ConnectionType.DEFAULT_USB_BAUD_RATE)
                    mavlinkConn = UsbConnection(context, baudRate)
                    Timber.i("Connecting over usb.")
                }
                ConnectionType.TYPE_BLUETOOTH -> {
                    //Retrieve the bluetooth address to connect to
                    val bluetoothAddress = paramsBundle!!.getString(ConnectionType.EXTRA_BLUETOOTH_ADDRESS)
                    mavlinkConn = BluetoothConnection(context, bluetoothAddress)
                    Timber.i("Connecting over bluetooth.")
                }
                ConnectionType.TYPE_TCP -> {
                    //Retrieve the server ip and port
                    val tcpServerIp = paramsBundle!!.getString(ConnectionType.EXTRA_TCP_SERVER_IP)
                    val tcpServerPort = paramsBundle.getInt(ConnectionType.EXTRA_TCP_SERVER_PORT, ConnectionType.DEFAULT_TCP_SERVER_PORT)
                    mavlinkConn = AndroidTcpConnection(context, tcpServerIp, tcpServerPort, WifiConnectionHandler(context))
                    Timber.i("Connecting over tcp.")
                }
                ConnectionType.TYPE_UDP -> {
                    paramsBundle?.getInt(ConnectionType.EXTRA_UDP_SERVER_PORT, ConnectionType.DEFAULT_UDP_SERVER_PORT)?.let { udpServerPort ->
                        mavlinkConn = AndroidUdpConnection(context, udpServerPort, WifiConnectionHandler(context))
                        Timber.i("Connecting over udp.")
                    }
                }
                ConnectionType.TYPE_SOLO -> {
                    Timber.i("Creating solo connection")
                    val soloLinkId = paramsBundle!!.getString(ConnectionType.EXTRA_SOLO_LINK_ID, null)
                    val linkPassword = paramsBundle.getString(ConnectionType.EXTRA_SOLO_LINK_PASSWORD, null)
                    mavlinkConn = SoloConnection(context, soloLinkId, linkPassword)
                }
                ConnectionType.TYPE_CUSTOM -> {
                    mavlinkConn = connParams.customConnection
                    Timber.i("Creating custom connection: %s", mavlinkConn!!.javaClass.name)
                }
                else -> {
                    Timber.e("Unrecognized connection type: %s", connectionType)
                    return
                }
            }
        }
        mavlinkConn!!.addMavLinkConnectionListener(tag, mConnectionListener)

        //Check if we need to ping a server to receive UDP data stream.
        if (connectionType == ConnectionType.TYPE_UDP) {
            val pingIpAddress = paramsBundle!!.getString(ConnectionType.EXTRA_UDP_PING_RECEIVER_IP)
            if (!TextUtils.isEmpty(pingIpAddress)) {
                try {
                    val resolvedAddress = InetAddress.getByName(pingIpAddress)
                    val pingPort = paramsBundle.getInt(ConnectionType.EXTRA_UDP_PING_RECEIVER_PORT)
                    val pingPeriod = paramsBundle.getLong(ConnectionType.EXTRA_UDP_PING_PERIOD,
                            ConnectionType.DEFAULT_UDP_PING_PERIOD)
                    val pingPayload = paramsBundle.getByteArray(ConnectionType.EXTRA_UDP_PING_PAYLOAD)
                    (mavlinkConn as AndroidUdpConnection?)!!.addPingTarget(resolvedAddress, pingPort, pingPeriod, pingPayload)
                } catch (e: UnknownHostException) {
                    Timber.e(e, "Unable to resolve UDP ping server ip address.")
                }
            }
        }
        if (mavlinkConn!!.connectionStatus == MavLinkConnection.MAVLINK_DISCONNECTED) {
            mavlinkConn!!.connect()
        }
    }

    /**
     * Disconnect the MAVLink connection for the given listener.
     */
    @Synchronized
    override fun closeConnection() {
        if (isDisconnected) return
        mavlinkConn!!.removeMavLinkConnectionListener(toString())
        if (mavlinkConn!!.mavLinkConnectionListenersCount == 0) {
            Timber.i("Disconnecting...")
            mavlinkConn!!.disconnect()
        }
        listener.onConnectionStatus(LinkConnectionStatus(LinkConnectionStatus.DISCONNECTED, null))
    }

    @Synchronized
    override fun sendMessage(message: MAVLinkMessage, listener: ICommandListener?) {
        sendMavMessage(message, DEFAULT_SYS_ID, DEFAULT_COMP_ID, listener)
    }

    protected open fun sendMavMessage(message: MAVLinkMessage?, sysId: Int, compId: Int, listener: ICommandListener?) {
        if (isDisconnected || message == null) {
            Timber.d("Not connected || message is null")
            return
        }

        // if(!(message instanceof msg_heartbeat)) {
        //     Log.v("SEND_MAVLINK", String.format("%s (%d)", message.getClass().getSimpleName(), message.msgid));
        // }
        val packet = message.pack()
        packet.sysid = sysId
        packet.compid = compId
        packet.seq = packetSeqNumber
        mavlinkConn!!.sendMavPacket(packet)
        packetSeqNumber = (packetSeqNumber + 1) % (MAX_PACKET_SEQUENCE + 1)
        if (commandTracker != null && listener != null) {
            commandTracker.onCommandSubmitted(message, listener)
        }
    }

    @get:Synchronized
    val isDisconnected: Boolean
        get() = connectionStatus == MavLinkConnection.MAVLINK_DISCONNECTED

    @get:Synchronized
    override val isConnected: Boolean
        get() = connectionStatus == MavLinkConnection.MAVLINK_CONNECTED
    private val isConnecting: Boolean
        private get() = connectionStatus == MavLinkConnection.MAVLINK_CONNECTING

    /**
     * Register a log listener.
     *
     * @param appId             Tag for the listener.
     */
    @Synchronized
    fun registerForTLogLogging(appId: String?, tlogLoggingUri: Uri?) {
        if (tlogLoggingUri == null) return
        if (isConnecting || isConnected) {
            val logFile = File(tlogLoggingUri.path)
            mavlinkConn!!.addLoggingPath(appId, logFile.absolutePath)
        }
    }

    /**
     * Unregister a log listener.
     *
     * @param appId        Tag for the listener.
     */
    @Synchronized
    fun unregisterForTLogLogging(appId: String?) {
        if (isConnecting || isConnected) {
            mavlinkConn!!.removeLoggingPath(appId)
        }
    }

    companion object {
        private const val DEFAULT_SYS_ID = 255
        private const val DEFAULT_COMP_ID = 190

        /**
         * Maximum possible sequence number for a packet.
         */
        private const val MAX_PACKET_SEQUENCE = 255
    }

    init {
        if (connParams == null) {
            throw NullPointerException("Invalid connection parameter argument.")
        }
        this.connParams = connParams
        this.commandTracker = commandTracker
    }
}
