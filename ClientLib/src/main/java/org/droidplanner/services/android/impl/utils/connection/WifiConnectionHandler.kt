package org.droidplanner.services.android.impl.utils.connection

import android.Manifest
import android.annotation.TargetApi
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.net.*
import android.net.ConnectivityManager.NetworkCallback
import android.net.NetworkInfo.DetailedState
import android.net.wifi.*
import android.os.Build
import android.provider.Settings
import android.support.v4.content.ContextCompat
import android.text.TextUtils
import android.widget.Toast
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus.Companion.newFailedConnectionStatus
import org.droidplanner.services.android.impl.utils.NetworkUtils.getCurrentWifiLink
import org.droidplanner.services.android.impl.utils.NetworkUtils.isSoloNetwork
import timber.log.Timber
import java.util.concurrent.atomic.AtomicReference

/**
 * Used to handle connection with the sololink wifi network.
 */
class WifiConnectionHandler(private val context: Context) {
    interface WifiConnectionListener {
        fun onWifiConnected(wifiSsid: String?)
        fun onWifiConnecting()
        fun onWifiDisconnected(prevConnectedSsid: String?)
        fun onWifiScanResultsAvailable(results: List<ScanResult>?)
        fun onWifiConnectionFailed(connectionStatus: LinkConnectionStatus?)
    }

    companion object {
        const val SOLO_LINK_WIFI_PREFIX = "SoloLink_"
        private val intentFilter = IntentFilter()
        private fun trimWifiSsid(wifiSsid: String?): String {
            return if (wifiSsid.isNullOrEmpty()) {
                ""
            } else wifiSsid!!.replace("\"", "")
        }

        fun getCurrentWifiLink(wifiMgr: WifiManager?): String {
            val connectedWifi = wifiMgr!!.connectionInfo
            val connectedSSID = connectedWifi?.ssid
            return trimWifiSsid(connectedSSID)
        }

        fun isSoloWifi(wifiSsid: String?): Boolean {
            return !TextUtils.isEmpty(wifiSsid) && wifiSsid!!.startsWith(SOLO_LINK_WIFI_PREFIX)
        }

        init {
            intentFilter.apply {
                addAction(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION)
                addAction(WifiManager.NETWORK_STATE_CHANGED_ACTION)
                addAction(WifiManager.WIFI_STATE_CHANGED_ACTION)
                addAction(WifiManager.SUPPLICANT_STATE_CHANGED_ACTION)
            }
        }
    }

    private val broadcastReceiver: BroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val action = intent.action
            when (action) {
                WifiManager.SCAN_RESULTS_AVAILABLE_ACTION -> notifyWifiScanResultsAvailable(
                    wifiScanResults
                )
                WifiManager.SUPPLICANT_STATE_CHANGED_ACTION -> {
                    val supState =
                        intent.getParcelableExtra<SupplicantState>(WifiManager.EXTRA_NEW_STATE)
                    val ssid = getCurrentWifiLink(context)
                    val supplicationError =
                        intent.getIntExtra(WifiManager.EXTRA_SUPPLICANT_ERROR, -1)
                    Timber.d(
                        "Supplicant state changed error %s with state %s and ssid %s",
                        supplicationError,
                        supState,
                        ssid
                    )
                    if (supplicationError == WifiManager.ERROR_AUTHENTICATING) {
                        if (isSoloNetwork(ssid)) {
                            notifyWifiConnectionFailed()
                            val wifiConfig = getWifiConfigs(ssid)
                            if (wifiConfig != null) {
                                wifiMgr!!.removeNetwork(wifiConfig.networkId)
                            }
                        }
                    }
                }
                WifiManager.NETWORK_STATE_CHANGED_ACTION -> {
                    val netInfo =
                        intent.getParcelableExtra<NetworkInfo>(WifiManager.EXTRA_NETWORK_INFO)
                    val networkState =
                        if (netInfo == null) NetworkInfo.State.DISCONNECTED else netInfo.state
                    when (networkState) {
                        NetworkInfo.State.CONNECTED -> {
                            val wifiInfo =
                                intent.getParcelableExtra<WifiInfo>(WifiManager.EXTRA_WIFI_INFO)
                            if (wifiInfo != null) {
                                val wifiSSID = wifiInfo.ssid
                                Timber.i("Connected to $wifiSSID")
                                val dhcpInfo = wifiMgr!!.dhcpInfo
                                if (dhcpInfo != null) {
                                    Timber.i("Dhcp info: %s", dhcpInfo.toString())
                                } else {
                                    Timber.w("Dhcp info is not available.")
                                }
                                wifiSSID?.let { setDefaultNetworkIfNecessary(it) }
                            } else {
                                Timber.w("No wifiInfo!!")
                            }
                        }
                        NetworkInfo.State.DISCONNECTED -> {
                            Timber.i("Disconnected from wifi network.")
                            notifyWifiDisconnected()
                        }
                        NetworkInfo.State.CONNECTING -> {
                            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                                val detailedState = netInfo!!.detailedState
                                if (detailedState != null && detailedState == DetailedState.VERIFYING_POOR_LINK) {
                                    val connectingSsid = getCurrentWifiLink(context)
                                    setDefaultNetworkIfNecessary(connectingSsid)
                                }
                            }
                            Timber.d("Connecting to wifi network.")
                            notifyWifiConnecting()
                        }
                    }
                }
                WifiManager.WIFI_STATE_CHANGED_ACTION -> {}
            }
        }
    }
    private var netReq: Any? = null
    private var netReqCb: Any? = null
    private val connectedWifi = AtomicReference("")
    private val wifiMgr: WifiManager?
    private val connMgr: ConnectivityManager?
    private var listener: WifiConnectionListener? = null
    fun setListener(listener: WifiConnectionListener?) {
        this.listener = listener
    }

    /**
     * Start the wifi connection handler process.
     * It will start listening for wifi connectivity updates, and will handle them as needed.
     */
    fun start() {
        context.registerReceiver(broadcastReceiver, intentFilter)
    }

    /**
     * Stop the wifi connection handler process.
     */
    fun stop() {
        try {
            context.unregisterReceiver(broadcastReceiver)
        } catch (e: IllegalArgumentException) {
            Timber.w(e, "Receiver was not registered.")
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            resetNetworkBindings(netReqCb as NetworkCallback?)
        }
    }

    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    private fun resetNetworkBindings(netCb: NetworkCallback?) {
        Timber.i("Unregistering network callbacks.")
        connectedWifi.set(getCurrentWifiLink(context))
        try {
            connMgr!!.unregisterNetworkCallback(netCb)
        } catch (e: IllegalArgumentException) {
            Timber.w(e, "Network callback was not registered.")
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            connMgr!!.bindProcessToNetwork(null)
        } else {
            ConnectivityManager.setProcessDefaultNetwork(null)
        }
    }

    /**
     * Query available wifi networks
     */
    fun refreshWifiAPs(): Boolean {
        Timber.d("Querying wifi access points.")
        if (wifiMgr == null) {
            return false
        }
        if (!wifiMgr.isWifiEnabled && !wifiMgr.setWifiEnabled(true)) {
            Toast.makeText(context, "Unable to activate Wi-Fi!", Toast.LENGTH_LONG).show()
            return false
        }
        return wifiMgr.startScan()
    }

    fun isOnNetwork(wifiSsid: String?): Boolean {
        require(!TextUtils.isEmpty(wifiSsid)) { "Invalid wifi ssid $wifiSsid" }
        return wifiSsid.equals(currentWifiLink, ignoreCase = true)
    }

    fun isConnected(wifiSSID: String?): Boolean {
        if (!isOnNetwork(wifiSSID)) {
            return false
        }
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            val network: Network?
            network = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                connMgr!!.boundNetworkForProcess
            } else {
                ConnectivityManager.getProcessDefaultNetwork()
            }
            if (network == null) {
                return false
            }
            val netCapabilities = connMgr!!.getNetworkCapabilities(network)
            netCapabilities != null && netCapabilities.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)
        } else {
            true
        }
    }

    val wifiScanResults: List<ScanResult>
        get() = if (hasLocationPermissions()) {
            wifiMgr!!.scanResults
        } else {
            ArrayList()
        }

    private fun hasNetworkChangeStatePermissions(): Boolean {
        var hasPerms = false
        for (perm in arrayOf(
            Manifest.permission.CHANGE_NETWORK_STATE
        )) {
            if (ContextCompat.checkSelfPermission(
                    context,
                    perm
                ) == PackageManager.PERMISSION_GRANTED
            ) {
                hasPerms = true
                break
            }
        }
        if (!hasPerms) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                hasPerms = Settings.System.canWrite(context)
            }
        }
        return hasPerms
    }

    private fun hasLocationPermissions(): Boolean {
        var hasPerms = false
        for (perm in arrayOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION
        )) {
            if (ContextCompat.checkSelfPermission(
                    context,
                    perm
                ) == PackageManager.PERMISSION_GRANTED
            ) {
                hasPerms = true
                break
            }
        }
        return hasPerms
    }

    fun connectToWifi(soloLinkId: String?, password: String): Int {
        if (TextUtils.isEmpty(soloLinkId)) {
            return LinkConnectionStatus.INVALID_CREDENTIALS
        }
        var targetScanResult: ScanResult? = null
        val scanResults = wifiScanResults
        for (result in scanResults) {
            if (result.SSID.equals(soloLinkId, ignoreCase = true)) {
                targetScanResult = result
                break
            }
        }
        if (targetScanResult == null) {
            Timber.i("No matching scan result was found for id %s", soloLinkId)
            return LinkConnectionStatus.LINK_UNAVAILABLE
        }
        return connectToWifi(targetScanResult, password)
    }

    fun connectToWifi(scanResult: ScanResult?, password: String): Int {
        if (scanResult == null) {
            return LinkConnectionStatus.LINK_UNAVAILABLE
        }
        Timber.d("Connecting to wifi " + scanResult.SSID)


        //Check if we're already connected to the given network.
        if (isConnected(scanResult.SSID)) {
            Timber.d("Already connected to " + scanResult.SSID)
            notifyWifiConnected(scanResult.SSID)
            return 0
        } else if (isOnNetwork(scanResult.SSID)) {
            setDefaultNetworkIfNecessary(scanResult.SSID)
            return 0
        }
        var wifiConfig = getWifiConfigs(scanResult.SSID)

        //Network is not configured and needs a password to connect
        if (wifiConfig == null) {
            Timber.d("Connecting to closed wifi network.")
            if (TextUtils.isEmpty(password)) {
                return LinkConnectionStatus.INVALID_CREDENTIALS
            }
            if (!connectToClosedWifi(scanResult, password)) {
                return LinkConnectionStatus.UNKNOWN
            }
            wifiMgr!!.saveConfiguration()
            wifiConfig = getWifiConfigs(scanResult.SSID)
        }
        if (wifiConfig != null) {
            wifiMgr!!.enableNetwork(wifiConfig.networkId, true)
            return 0
        }
        return LinkConnectionStatus.UNKNOWN
    }

    private fun getWifiConfigs(networkSSID: String?): WifiConfiguration? {
        val networks = wifiMgr!!.configuredNetworks ?: return null
        for (current in networks) {
            if (current.SSID != null && current.SSID == "\"" + networkSSID + "\"") {
                return current
            }
        }
        return null
    }

    private fun connectToClosedWifi(scanResult: ScanResult, password: String): Boolean {
        val wifiConf = WifiConfiguration()
        wifiConf.SSID =
            "\"" + scanResult.SSID + "\"" //Please note the quotes. String should contain ssid in quotes.
        wifiConf.preSharedKey = "\"" + password + "\""
        val netId = wifiMgr!!.addNetwork(wifiConf)
        if (netId == -1) {
            Timber.e("Unable to add wifi configuration for %s", scanResult.SSID)
            return false
        }
        return true
    }

    private val currentWifiLink: String
        private get() = getCurrentWifiLink(wifiMgr)

    private fun setDefaultNetworkIfNecessary(wifiSsid: String?) {
        val trimmedSsid = trimWifiSsid(wifiSsid)
        if (trimmedSsid != connectedWifi.get()) {
            connectedWifi.set(trimmedSsid)
            if (isConnected(wifiSsid)) {
                notifyWifiConnected(wifiSsid)
                return
            }
            if (isSoloWifi(trimmedSsid)) {
                //Attempt to connect to the vehicle.
                Timber.i("Requesting route to sololink network")
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                    if (hasNetworkChangeStatePermissions()) {
                        connMgr!!.requestNetwork(
                            netReq as NetworkRequest?,
                            netReqCb as NetworkCallback?
                        )
                    }
                } else {
                    notifyWifiConnected(trimmedSsid)
                }
            } else {
                notifyWifiConnected(trimmedSsid)
            }
        }
    }

    private fun notifyWifiConnected(wifiSsid: String?) {
        if (listener != null) {
            listener!!.onWifiConnected(wifiSsid)
        }
    }

    private fun notifyWifiConnecting() {
        if (listener != null) {
            listener!!.onWifiConnecting()
        }
    }

    private fun notifyWifiDisconnected() {
        if (listener != null) {
            listener!!.onWifiDisconnected(connectedWifi.get())
        }
        connectedWifi.set("")
    }

    private fun notifyWifiScanResultsAvailable(results: List<ScanResult>) {
        if (listener != null) {
            listener!!.onWifiScanResultsAvailable(results)
        }
    }

    private fun notifyWifiConnectionFailed() {
        if (listener != null) {
            val linkConnectionStatus =
                newFailedConnectionStatus(LinkConnectionStatus.INVALID_CREDENTIALS, null)
            listener!!.onWifiConnectionFailed(linkConnectionStatus)
        }
    }

    init {
        wifiMgr = context.getSystemService(Context.WIFI_SERVICE) as WifiManager
        connMgr = context.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            netReq = NetworkRequest.Builder()
                .addCapability(NetworkCapabilities.NET_CAPABILITY_TRUSTED)
                .addCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
                .addCapability(NetworkCapabilities.NET_CAPABILITY_NOT_RESTRICTED)
                .addCapability(NetworkCapabilities.NET_CAPABILITY_NOT_VPN)
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .build()
            netReqCb = object : NetworkCallback() {
                @TargetApi(Build.VERSION_CODES.LOLLIPOP)
                private fun getNetworkInfo(network: Network?) {
                    if (network == null) {
                        Timber.i("Network is null.")
                    } else if (connMgr != null) {
                        Timber.i(
                            "Network: %s, active : %s",
                            network,
                            connMgr.isDefaultNetworkActive
                        )
                        val linkProps = connMgr.getLinkProperties(network)
                        Timber.i("Network link properties: %s", linkProps.toString())
                        Timber.i(
                            "Network capabilities: %s",
                            connMgr.getNetworkCapabilities(network)
                        )
                    }
                }

                @TargetApi(Build.VERSION_CODES.LOLLIPOP)
                override fun onAvailable(network: Network) {
                    //Check if we're still connected to solo. If not, unregister the callbacks
                    val currentWifi = currentWifiLink
                    if (!isSoloWifi(currentWifi)) {
                        resetNetworkBindings(this)
                        return
                    }
                    Timber.i("Network %s is available", network)
                    getNetworkInfo(network)
                    val wasBound: Boolean
                    wasBound = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                        connMgr!!.bindProcessToNetwork(network)
                    } else {
                        ConnectivityManager.setProcessDefaultNetwork(network)
                    }
                    if (wasBound) {
                        Timber.i("Bound process to network %s", network)
                        notifyWifiConnected(currentWifi)
                    } else {
                        Timber.w("Unable to bind process to network %s", network)
                    }
                }

                override fun onLosing(network: Network, maxMsToLive: Int) {
                    Timber.w("Losing network %s", network)
                }

                override fun onLost(network: Network) {
                    Timber.w("Lost network %s", network)
                }
            }
        } else {
            netReq = null
            netReqCb = null
        }
    }
}
