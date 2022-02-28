package org.droidplanner.services.android.impl.utils

import android.content.Context
import org.droidplanner.services.android.impl.utils.NetworkUtils
import android.net.ConnectivityManager
import android.net.NetworkInfo
import android.net.wifi.WifiManager
import android.net.wifi.WifiInfo
import com.o3dr.android.client.BuildConfig
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.SoloComp

/**
 * Created by Fredia Huya-Kouadio on 5/11/15.
 */
object NetworkUtils {
    /**
     * Is internet connection available. This method also returns true for the SITL build type
     * @param context
     * @return Internet connection availability.
     */
    fun isNetworkAvailable(context: Context): Boolean {
        if (!BuildConfig.SITL_DEBUG && isOnSololinkNetwork(context)) return false
        val connectivityManager = context
                .getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        val activeNetworkInfo = connectivityManager.activeNetworkInfo
        return activeNetworkInfo != null && activeNetworkInfo.isConnected
    }

    @JvmStatic
    fun getCurrentWifiLink(context: Context): String? {
        val wifiMgr = context.getSystemService(Context.WIFI_SERVICE) as WifiManager
        val connectedWifi = wifiMgr.connectionInfo
        return connectedWifi?.ssid?.replace("\"", "")
    }

    fun isOnSololinkNetwork(context: Context): Boolean {
        if (BuildConfig.SITL_DEBUG) return true
        val connectedSSID = getCurrentWifiLink(context)
        return isSoloNetwork(connectedSSID)
    }

    @JvmStatic
    fun isSoloNetwork(ssid: String?): Boolean {
        return ssid != null && ssid.startsWith(SoloComp.SOLO_LINK_WIFI_PREFIX)
    }
}
