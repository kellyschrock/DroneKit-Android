package org.droidplanner.services.android.impl.communication.connection

import android.content.Context
import android.net.Uri
import android.net.wifi.ScanResult
import android.net.wifi.WifiManager
import android.text.TextUtils
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter.Companion.newSoloConnection
import com.o3dr.services.android.lib.drone.connection.ConnectionType
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus.Companion.newFailedConnectionStatus
import org.droidplanner.services.android.impl.utils.connection.WifiConnectionHandler
import org.droidplanner.services.android.impl.utils.connection.WifiConnectionHandler.WifiConnectionListener
import timber.log.Timber
import java.io.IOException

/**
 * Abstract the connection to a Solo vehicle.
 * Created by Fredia Huya-Kouadio on 12/17/15.
 */
class SoloConnection(
        applicationContext: Context,
        private val soloLinkId: String,
        private val soloLinkPassword: String)
: AndroidMavLinkConnection(applicationContext), WifiConnectionListener {

    private val wifiHandler: WifiConnectionHandler = WifiConnectionHandler(applicationContext)
    private val dataLink: AndroidUdpConnection = object : AndroidUdpConnection(applicationContext, SOLO_UDP_PORT) {
        override fun onConnectionOpened() {
            this@SoloConnection.onConnectionOpened()
        }

        override fun onConnectionStatus(connectionStatus: LinkConnectionStatus) {
            this@SoloConnection.onConnectionStatus(connectionStatus)
        }
    }

    @Throws(IOException::class)
    override fun openConnection() {
        if (TextUtils.isEmpty(soloLinkId)) {
            val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.INVALID_CREDENTIALS, "Invalid connection credentials!")
            onConnectionStatus(connectionStatus)
        } else {
            wifiHandler.start()
            checkScanResults(wifiHandler.wifiScanResults)
        }
    }

    private fun refreshWifiAps() {
        if (!wifiHandler.refreshWifiAPs()) {
            val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.SYSTEM_UNAVAILABLE, "Unable to refresh wifi access points")
            onConnectionStatus(connectionStatus)
        }
    }

    @Throws(IOException::class)
    override fun readDataBlock(buffer: ByteArray): Int {
        return dataLink.readDataBlock(buffer)
    }

    @Throws(IOException::class)
    override fun sendBuffer(buffer: ByteArray) {
        dataLink.sendBuffer(buffer)
    }

    @Throws(IOException::class)
    override fun closeConnection() {
        wifiHandler.stop()
        dataLink.closeConnection()
    }

    override fun loadPreferences() {
        dataLink.loadPreferences()
    }

    override fun getConnectionType(): Int {
        return dataLink.connectionType
    }

    override fun onWifiConnected(wifiSsid: String) {
        if (isConnecting) {
            //Let's see if we're connected to our target wifi
            if (wifiSsid.equals(soloLinkId, ignoreCase = true)) {
                //We're good to go
                try {
                    dataLink.openConnection()
                } catch (e: IOException) {
                    reportIOException(e)
                    Timber.e(e, e.message)
                }
            }
        }
    }

    override fun onWifiConnecting() {
        onConnectionStatus(LinkConnectionStatus(LinkConnectionStatus.CONNECTING, null))
    }

    override fun onWifiDisconnected(prevSsid: String) {
        if (prevSsid.equals(soloLinkId, ignoreCase = true)) {
            onConnectionStatus(LinkConnectionStatus(LinkConnectionStatus.DISCONNECTED, null))
        }
    }

    override fun onWifiScanResultsAvailable(results: List<ScanResult>) {
        checkScanResults(results)
    }

    override fun onWifiConnectionFailed(connectionStatus: LinkConnectionStatus) {
        onConnectionStatus(connectionStatus)
    }

    private fun checkScanResults(results: List<ScanResult>?) {
        if (!isConnecting) {
            return
        }
        if (results == null) {
            return
        }

        //We're in the connection process, let's see if the wifi we want is available
        var targetResult: ScanResult? = null
        for (result in results) {
            if (result.SSID.equals(soloLinkId, ignoreCase = true)) {
                //bingo
                targetResult = result
                break
            }
        }
        if (targetResult != null) {
            //We're good to go
            try {
                val connectionResult = wifiHandler.connectToWifi(targetResult, soloLinkPassword)
                if (connectionResult != 0) {
                    val connectionStatus = newFailedConnectionStatus(connectionResult, "Unable to connect to the target wifi $soloLinkId")
                    onConnectionStatus(connectionStatus)
                }
            } catch (e: IllegalArgumentException) {
                Timber.e(e, e.message)
                val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.UNKNOWN, e.message)
                onConnectionStatus(connectionStatus)
            }
        } else {
            //Let's try again
            refreshWifiAps()
        }
    }

    private val isConnecting: Boolean
        private get() = connectionStatus == MAVLINK_CONNECTING

    companion object {
        private const val SOLO_UDP_PORT = 14550
        @JvmStatic
        fun isUdpSoloConnection(context: Context, connParam: ConnectionParameter?): Boolean {
            if (connParam == null) return false
            val connectionType = connParam.connectionType
            return when (connectionType) {
                ConnectionType.TYPE_UDP -> {
                    val paramsBundle = connParam.paramsBundle ?: return false
                    val serverPort = paramsBundle.getInt(ConnectionType.EXTRA_UDP_SERVER_PORT, ConnectionType.DEFAULT_UDP_SERVER_PORT)
                    val wifiSsid = WifiConnectionHandler.getCurrentWifiLink(context.getSystemService(Context.WIFI_SERVICE) as WifiManager)
                    WifiConnectionHandler.isSoloWifi(wifiSsid) && serverPort == SOLO_UDP_PORT
                }
                else -> false
            }
        }

        fun getSoloConnectionParameterFromUdp(context: Context?, tLogLoggingUri: Uri?): ConnectionParameter? {
            if (context == null) return null
            val wifiSsid = WifiConnectionHandler.getCurrentWifiLink(context.getSystemService(Context.WIFI_SERVICE) as WifiManager)
            return if (WifiConnectionHandler.isSoloWifi(wifiSsid)) {
                newSoloConnection(wifiSsid, null, tLogLoggingUri)
            } else null
        }

        @JvmStatic
        fun getSoloConnectionParameterFromUdp(context: Context?, params: ConnectionParameter?): ConnectionParameter? {
            if (context == null) return null
            val wifiSsid = WifiConnectionHandler.getCurrentWifiLink(context.getSystemService(Context.WIFI_SERVICE) as WifiManager)
            return if (WifiConnectionHandler.isSoloWifi(wifiSsid)) {
                newSoloConnection(wifiSsid, null, params!!)
            } else null
        }
    }

    init {
        wifiHandler.setListener(this)
    }
}
