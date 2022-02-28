package org.droidplanner.services.android.impl.communication.connection

import android.content.Context
import android.util.Log
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import org.droidplanner.services.android.impl.core.MAVLink.connection.UdpConnection
import org.droidplanner.services.android.impl.core.model.Logger
import org.droidplanner.services.android.impl.utils.connection.WifiConnectionHandler
import timber.log.Timber
import java.io.IOException
import java.net.InetAddress
import java.util.*
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

private val TAG = AndroidUdpConnection::class.java.simpleName

open class AndroidUdpConnection @JvmOverloads constructor(
        context: Context,
        private val serverPort: Int,
        wifiHandler: WifiConnectionHandler? = null)
: AndroidIpConnection(context, wifiHandler) {

    private val connectionImpl: UdpConnection = object : UdpConnection() {
        override fun loadServerPort(): Int {
            return serverPort
        }

        override fun initLogger(): Logger {
            return this@AndroidUdpConnection.initLogger()
        }

        override fun onConnectionOpened() {
            Timber.d("onConnectionOpened()")
            this@AndroidUdpConnection.onConnectionOpened()
        }

        override fun onConnectionStatus(connectionStatus: LinkConnectionStatus) {
            Timber.d("onConnectionStatus(): %s", connectionStatus)
            this@AndroidUdpConnection.onConnectionStatus(connectionStatus)
        }
    }

    private val pingTasks = HashSet<PingTask>()
    private var pingRunner: ScheduledExecutorService? = null

    fun addPingTarget(address: InetAddress?, port: Int, period: Long, payload: ByteArray?) {
        Timber.d("addPingTarget(%s, %d, %d, %s)", address, port, period, payload)

        if (address == null || payload == null || period <= 0) return
        val pingTask = PingTask(address, port, period, payload)
        pingTasks.add(pingTask)
        if (connectionStatus == MAVLINK_CONNECTED && pingRunner != null && !pingRunner!!.isShutdown) pingRunner!!.scheduleWithFixedDelay(pingTask, 0, period, TimeUnit.MILLISECONDS)
    }

    @Throws(IOException::class)
    override fun onCloseConnection() {
        Log.d(TAG, "Closing udp connection.")

        Log.d(TAG, "Shutting down pinging tasks.")
        pingRunner?.shutdownNow()
        pingRunner = null
        connectionImpl.closeConnection()
    }

    override fun loadPreferences() {
        connectionImpl.loadPreferences()
    }

    @Throws(IOException::class)
    override fun onOpenConnection() {
        Log.d(TAG, "Opening udp connection")
        connectionImpl.openConnection()
        if (pingRunner == null || pingRunner!!.isShutdown) pingRunner = Executors.newSingleThreadScheduledExecutor()
        for (pingTask in pingTasks) pingRunner!!.scheduleWithFixedDelay(pingTask, 0, pingTask.period, TimeUnit.MILLISECONDS)
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

    inner class PingTask(private val address: InetAddress, private val port: Int, val period: Long, private val payload: ByteArray) : Runnable {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (other !is PingTask) return false
            return address == other.address && port == other.port && period == other.period
        }

        override fun hashCode(): Int {
            return toString().hashCode()
        }

        override fun run() {
            try {
                connectionImpl.sendBuffer(address, port, payload)
            } catch (e: IOException) {
                Log.e(TAG, "Error occurred while sending ping message.", e)
            }
        }

        override fun toString(): String {
            return "[$address; $port; $period]"
        }
    }
}
