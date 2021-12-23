package org.droidplanner.services.android.impl.communication.connection

import android.content.Context
import org.droidplanner.services.android.impl.core.MAVLink.connection.MavLinkConnection
import org.droidplanner.services.android.impl.core.model.Logger
import org.droidplanner.services.android.impl.utils.AndroidLogger

abstract class AndroidMavLinkConnection(val context: Context) : MavLinkConnection() {
    override fun initLogger(): Logger {
        return AndroidLogger.getLogger()
    }
}
