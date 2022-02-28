package org.droidplanner.services.android.impl.communication.connection

import android.content.Context
import kotlin.jvm.JvmOverloads
import org.droidplanner.services.android.impl.utils.connection.WifiConnectionHandler
import kotlin.Throws
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import org.droidplanner.services.android.impl.core.MAVLink.connection.TcpConnection
import org.droidplanner.services.android.impl.core.model.Logger
import java.io.IOException

class AndroidTcpConnection @JvmOverloads constructor(context: Context, private val serverIp: String, private val serverPort: Int, wifiHandler: WifiConnectionHandler? = null)
: AndroidIpConnection(context, wifiHandler) {
    private val connectionImpl: TcpConnection = object : TcpConnection() {
        override fun loadServerPort(): Int {
            return serverPort
        }

        override fun loadServerIP(): String {
            return serverIp
        }

        override fun initLogger(): Logger {
            return this@AndroidTcpConnection.initLogger()
        }

        override fun onConnectionOpened() {
            this@AndroidTcpConnection.onConnectionOpened()
        }

        override fun onConnectionStatus(connectionStatus: LinkConnectionStatus) {
            this@AndroidTcpConnection.onConnectionStatus(connectionStatus)
        }
    }

    @Throws(IOException::class)
    override fun onCloseConnection() {
        connectionImpl.closeConnection()
    }

    override fun loadPreferences() {
        connectionImpl.loadPreferences()
    }

    @Throws(IOException::class)
    override fun onOpenConnection() {
        connectionImpl.openConnection()
    }

    @Throws(IOException::class)
    override fun readDataBlock(buffer: ByteArray): Int {
        return connectionImpl.readDataBlock(buffer)
    }

    @Throws(IOException::class)
    override fun sendBuffer(buffer: ByteArray) {
        connectionImpl.sendBuffer(buffer)
    }

    override fun getConnectionType(): Int {
        return connectionImpl.connectionType
    }
}
