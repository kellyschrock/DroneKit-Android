package org.droidplanner.services.android.impl.core.drone

/**
 * Created by Fredia Huya-Kouadio on 3/23/15.
 */
interface LogMessageListener {
    fun onMessageLogged(logLevel: Int, message: String)
}
