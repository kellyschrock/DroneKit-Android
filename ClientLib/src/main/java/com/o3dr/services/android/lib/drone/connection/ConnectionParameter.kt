package com.o3dr.services.android.lib.drone.connection

import android.net.Uri
import android.os.Parcelable
import android.os.Bundle
import org.droidplanner.services.android.impl.communication.connection.AndroidMavLinkConnection
import com.o3dr.services.android.lib.drone.connection.ConnectionType
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import android.os.Parcel
import android.text.TextUtils
import timber.log.Timber

/**
 * Base type used to pass the drone connection parameters over ipc.
 */
class ConnectionParameter : Parcelable, Cloneable {
    @get:ConnectionType.Type
    @ConnectionType.Type
    val connectionType: Int
    val paramsBundle: Bundle?

    /**
     * Return the uri where the tlog data should be logged.
     * @return Uri where to log the tlog data, or null if it shouldn't be logged.
     */
    val tLogLoggingUri: Uri?
    val customConnection: AndroidMavLinkConnection?

    /**
     */
    private constructor(@ConnectionType.Type connectionType: Int, paramsBundle: Bundle?, tlogLoggingUri: Uri? = null, customConnection: AndroidMavLinkConnection? = null) {
        this.connectionType = connectionType
        this.paramsBundle = paramsBundle
        tLogLoggingUri = tlogLoggingUri
        this.customConnection = customConnection
    }

    val uniqueId: String
        get() {
            val uniqueId: String
            when (connectionType) {
                ConnectionType.TYPE_UDP -> {
                    var udpPort = ConnectionType.DEFAULT_UDP_SERVER_PORT
                    if (paramsBundle != null) {
                        udpPort = paramsBundle.getInt(ConnectionType.EXTRA_UDP_SERVER_PORT, udpPort)
                    }
                    uniqueId = "udp:$udpPort"
                }
                ConnectionType.TYPE_BLUETOOTH -> {
                    var btAddress = ""
                    if (paramsBundle != null) {
                        btAddress = paramsBundle.getString(ConnectionType.EXTRA_BLUETOOTH_ADDRESS, "")
                    }
                    uniqueId = if (TextUtils.isEmpty(btAddress)) "bluetooth" else "bluetooth:$btAddress"
                }
                ConnectionType.TYPE_TCP -> {
                    var tcpIp = ""
                    var tcpPort = ConnectionType.DEFAULT_TCP_SERVER_PORT
                    if (paramsBundle != null) {
                        tcpIp = paramsBundle.getString(ConnectionType.EXTRA_TCP_SERVER_IP, "")
                        tcpPort = paramsBundle.getInt(ConnectionType.EXTRA_TCP_SERVER_PORT, tcpPort)
                    }
                    uniqueId = "tcp:$tcpIp:$tcpPort"
                }
                ConnectionType.TYPE_USB -> uniqueId = "usb"
                ConnectionType.TYPE_SOLO -> {
                    var soloLinkId = ""
                    if (paramsBundle != null) {
                        soloLinkId = paramsBundle.getString(ConnectionType.EXTRA_SOLO_LINK_ID, "")
                    }
                    uniqueId = "solo:$soloLinkId"
                }
                ConnectionType.TYPE_CUSTOM -> uniqueId = "custom: " + paramsBundle!!.getString(ConnectionType.EXTRA_CUSTOM_CONNECTION_ID, "")
                else -> uniqueId = ""
            }
            return uniqueId
        }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is ConnectionParameter) return false
        return uniqueId == o.uniqueId
    }

    override fun hashCode(): Int {
        return uniqueId.hashCode()
    }

    override fun toString(): String {
        var toString = "ConnectionParameter{" +
                "connectionType=" + connectionType +
                ", paramsBundle=["
        if (paramsBundle != null && !paramsBundle.isEmpty) {
            var isFirst = true
            for (key in paramsBundle.keySet()) {
                if (isFirst) isFirst = false else toString += ", "
                toString += key + "=" + paramsBundle[key]
            }
        }
        toString += "]}"
        return toString
    }

    public override fun clone(): ConnectionParameter {
        return ConnectionParameter(connectionType, paramsBundle)
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(connectionType)
        dest.writeBundle(paramsBundle)
        dest.writeParcelable(tLogLoggingUri, flags)
    }

    private constructor(`in`: Parcel) {
        @ConnectionType.Type val type = `in`.readInt()
        connectionType = type
        paramsBundle = `in`.readBundle(javaClass.classLoader)
        tLogLoggingUri = `in`.readParcelable(Uri::class.java.classLoader)
        customConnection = null
    }

    companion object {
        /**
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns a new [ConnectionParameter] with type [ConnectionType.TYPE_USB]
         * and baud rate [ConnectionType.DEFAULT_USB_BAUD_RATE].
         */
        @JvmStatic
        fun newUsbConnection(tlogLoggingUri: Uri?): ConnectionParameter {
            return newUsbConnection(ConnectionType.DEFAULT_USB_BAUD_RATE, tlogLoggingUri)
        }

        /**
         *
         * @param usbBaudRate Baud rate for USB connection.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns a new [ConnectionParameter] with type [ConnectionType.TYPE_USB].
         */
        @JvmStatic
        fun newUsbConnection(usbBaudRate: Int, tlogLoggingUri: Uri?): ConnectionParameter {
            val paramsBundle = Bundle(1)
            paramsBundle.putInt(ConnectionType.EXTRA_USB_BAUD_RATE, usbBaudRate)
            return ConnectionParameter(ConnectionType.TYPE_USB, paramsBundle, tlogLoggingUri, null)
        }

        /**
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_UDP], using
         * [ConnectionType.DEFAULT_UDP_SERVER_PORT] port.
         */
        @JvmStatic
        fun newUdpConnection(tlogLoggingUri: Uri?): ConnectionParameter {
            return newUdpConnection(ConnectionType.DEFAULT_UDP_SERVER_PORT, tlogLoggingUri)
        }

        /**
         *
         * @param udpPort Port for the UDP connection.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_UDP].
         */
        @JvmStatic
        fun newUdpConnection(udpPort: Int, tlogLoggingUri: Uri?): ConnectionParameter {
            return newUdpConnection(null, udpPort, null, 0, null, tlogLoggingUri)
        }

        /**
         * @param udpIP IP addresss for the UDP connection.
         * @param udpPort Port for the UDP connection.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_UDP].
         */
        @JvmStatic
        fun newUdpConnection(udpIP: String?, udpPort: Int, tlogLoggingUri: Uri?): ConnectionParameter {
            return newUdpConnection(udpIP, udpPort, null, 0, null, tlogLoggingUri)
        }

        /**
         *
         * @param udpIP IP address for the UDP connection.
         * @param udpPort Port for the UDP connection.
         * @param udpPingReceiverIp IP address of the UDP server to ping. If this value is null, it is ignored
         * along with udpPingReceiverPort and udpPingPayload.
         * @param udpPingReceiverPort Port of the UDP server to ping.
         * @param udpPingPayload Ping payload.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         *
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_UDP]. The ping
         * period is set to [ConnectionType.DEFAULT_UDP_PING_PERIOD]
         */
        @JvmStatic
        fun newUdpConnection(udpIP: String?, udpPort: Int, udpPingReceiverIp: String?, udpPingReceiverPort: Int,
                             udpPingPayload: ByteArray?, tlogLoggingUri: Uri?): ConnectionParameter {
            return newUdpConnection(udpIP, udpPort, udpPingReceiverIp, udpPingReceiverPort, udpPingPayload, ConnectionType.DEFAULT_UDP_PING_PERIOD, tlogLoggingUri)
        }

        /**
         *
         * @param udpIP IP address for the UDP connection.
         * @param udpPort Port for the UDP connection.
         * @param udpPingReceiverIp IP address of the UDP server to ping. If this value is null, it is ignored
         * along with udpPingReceiverPort, udpPingPayload, and pingPeriod.
         * @param udpPingReceiverPort Port of the UDP server to ping.
         * @param udpPingPayload Ping payload.
         * @param pingPeriod How often should the udp ping be performed.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         *
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_UDP].
         */
        @JvmStatic
        fun newUdpConnection(udpIP: String?, udpPort: Int, udpPingReceiverIp: String?, udpPingReceiverPort: Int,
                             udpPingPayload: ByteArray?, pingPeriod: Long, tlogLoggingUri: Uri?): ConnectionParameter {
            val paramsBundle = Bundle()
            paramsBundle.putInt(ConnectionType.EXTRA_UDP_SERVER_PORT, udpPort)
            Timber.d("newUdpConnection(%s, %d, %s, %d, %s, %d, %s)",
                    udpIP, udpPort, udpPingReceiverIp, udpPingReceiverPort, udpPingPayload, pingPeriod, tlogLoggingUri)
            if (!TextUtils.isEmpty(udpIP)) {
                paramsBundle.putString(ConnectionType.EXTRA_UDP_SERVER_IP, udpIP)
            }
            if (!TextUtils.isEmpty(udpPingReceiverIp)) {
                paramsBundle.putString(ConnectionType.EXTRA_UDP_PING_RECEIVER_IP, udpPingReceiverIp)
                paramsBundle.putInt(ConnectionType.EXTRA_UDP_PING_RECEIVER_PORT, udpPingReceiverPort)
                paramsBundle.putByteArray(ConnectionType.EXTRA_UDP_PING_PAYLOAD, udpPingPayload)
                paramsBundle.putLong(ConnectionType.EXTRA_UDP_PING_PERIOD, pingPeriod)
            }
            return ConnectionParameter(ConnectionType.TYPE_UDP, paramsBundle, tlogLoggingUri, null)
        }

        /**
         *
         * @param tcpServerIp TCP server IP address.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_TCP], using
         * [ConnectionType.DEFAULT_TCP_SERVER_PORT].
         */
        @JvmStatic
        fun newTcpConnection(tcpServerIp: String?, tlogLoggingUri: Uri?): ConnectionParameter {
            return newTcpConnection(tcpServerIp, ConnectionType.DEFAULT_TCP_SERVER_PORT, tlogLoggingUri)
        }

        /**
         *
         * @param tcpServerIp TCP server IP address.
         * @param tcpServerPort TCP server port.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_TCP].
         */
        @JvmStatic
        fun newTcpConnection(tcpServerIp: String?, tcpServerPort: Int, tlogLoggingUri: Uri?): ConnectionParameter {
            val paramsBundle = Bundle(2)
            paramsBundle.putString(ConnectionType.EXTRA_TCP_SERVER_IP, tcpServerIp)
            paramsBundle.putInt(ConnectionType.EXTRA_TCP_SERVER_PORT, tcpServerPort)
            return ConnectionParameter(ConnectionType.TYPE_TCP, paramsBundle, tlogLoggingUri, null)
        }

        /**
         *
         * @param bluetoothAddress Bluetooth address.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_BLUETOOTH].
         */
        @JvmStatic
        fun newBluetoothConnection(bluetoothAddress: String?, tlogLoggingUri: Uri?): ConnectionParameter {
            val paramsBundle = Bundle(1)
            paramsBundle.putString(ConnectionType.EXTRA_BLUETOOTH_ADDRESS, bluetoothAddress)
            return ConnectionParameter(ConnectionType.TYPE_BLUETOOTH, paramsBundle, tlogLoggingUri, null)
        }

        @JvmStatic
        fun newCustomConnection(connection: AndroidMavLinkConnection?, connectionId: String?): ConnectionParameter {
            val paramsBundle = Bundle(1)
            paramsBundle.putString(ConnectionType.EXTRA_CUSTOM_CONNECTION_ID, connectionId)
            return ConnectionParameter(ConnectionType.TYPE_CUSTOM, paramsBundle, null, connection)
        }

        @JvmStatic
        fun newCustomConnection(connection: AndroidMavLinkConnection?, connectionId: String?, tlogUri: Uri?): ConnectionParameter {
            val paramsBundle = Bundle(1)
            paramsBundle.putString(ConnectionType.EXTRA_CUSTOM_CONNECTION_ID, connectionId)
            return ConnectionParameter(ConnectionType.TYPE_CUSTOM, paramsBundle, tlogUri, connection)
        }

        /**
         *
         * @param ssid Wifi SSID of the solo vehicle link. This will remove a leading and/or trailing quotation.
         * @param password Password to access the solo wifi network. This value can be null as long as the wifi
         * configuration has been set up and stored in the mobile device's system.
         * @param tlogLoggingUri Uri where the tlog data should be logged. Pass null if the tlog data shouldn't be logged.
         * @return Returns [ConnectionParameter] with type [ConnectionType.TYPE_SOLO].
         */
        @JvmStatic
        fun newSoloConnection(ssid: String, password: String?, tlogLoggingUri: Uri?): ConnectionParameter {
            val ssidWithoutQuotes = ssid.replace("^\"|\"$".toRegex(), "")
            val paramsBundle = Bundle(2)
            paramsBundle.putString(ConnectionType.EXTRA_SOLO_LINK_ID, ssidWithoutQuotes)
            paramsBundle.putString(ConnectionType.EXTRA_SOLO_LINK_PASSWORD, password)
            return ConnectionParameter(ConnectionType.TYPE_SOLO, paramsBundle, tlogLoggingUri, null)
        }

        @JvmStatic
        fun newSoloConnection(ssid: String, password: String?, params: ConnectionParameter): ConnectionParameter {
            val ssidWithoutQuotes = ssid.replace("^\"|\"$".toRegex(), "")
            val paramsBundle = Bundle(params.paramsBundle)
            paramsBundle.putString(ConnectionType.EXTRA_SOLO_LINK_ID, ssidWithoutQuotes)
            paramsBundle.putString(ConnectionType.EXTRA_SOLO_LINK_PASSWORD, password)
            return ConnectionParameter(ConnectionType.TYPE_SOLO, paramsBundle, params.tLogLoggingUri, null)
        }

        @JvmField
        val CREATOR: Parcelable.Creator<ConnectionParameter> = object : Parcelable.Creator<ConnectionParameter> {
            override fun createFromParcel(source: Parcel): ConnectionParameter? {
                return ConnectionParameter(source)
            }

            override fun newArray(size: Int): Array<ConnectionParameter?> {
                return arrayOfNulls(size)
            }
        }
    }
}
