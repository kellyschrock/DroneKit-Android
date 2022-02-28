package org.droidplanner.services.android.impl.communication.connection

import android.content.Context
import org.droidplanner.services.android.impl.utils.connection.WifiConnectionHandler
import org.droidplanner.services.android.impl.communication.connection.AndroidMavLinkConnection
import java.io.IOException
import kotlin.Throws

/**
 * Created by fredia on 3/28/16.
 */
abstract class AndroidIpConnection protected constructor(
        context: Context, private val wifiHandler: WifiConnectionHandler?)
: AndroidMavLinkConnection(context!!) {
    constructor(applicationContext: Context) : this(applicationContext, null) {}

    @Throws(IOException::class)
    override fun openConnection() {
        wifiHandler?.start()
        onOpenConnection()
    }

    @Throws(IOException::class)
    protected abstract fun onOpenConnection()
    @Throws(IOException::class)
    override fun closeConnection() {
        onCloseConnection()
        wifiHandler?.stop()
    }

    @Throws(IOException::class)
    protected abstract fun onCloseConnection()
}
