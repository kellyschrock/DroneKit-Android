package com.o3dr.android.client.utils.connection

import android.os.Handler
import android.os.Process
import android.os.RemoteException
import com.o3dr.services.android.lib.model.ICommandListener
import timber.log.Timber
import java.io.IOException
import java.io.InterruptedIOException
import java.nio.ByteBuffer
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.atomic.AtomicInteger

private val TAG = AbstractIpConnection::class.java.simpleName

/** Base class for ip connection (tcp, udp). */
abstract class AbstractIpConnection(
        private val handler: Handler?,
        readBufferSize: Int,
        disableSending: Boolean,
        disableReading: Boolean,
        polling: Boolean) {
    private var ipConnectionListener: IpConnectionListener? = null

    /**
     * Queue the set of packets to send.
     * A thread will be blocking on it until there's element(s) available to send.
     */
    private val packetsToSend = LinkedBlockingQueue<PacketData>()
    private val connectionStatus = AtomicInteger(STATE_DISCONNECTED)
    private val isSendingDisabled: Boolean = disableSending
    private val isReadingDisabled: Boolean = disableReading
    private val readBuffer: ByteBuffer = ByteBuffer.allocate(readBufferSize)

    private val managerTask = Runnable {
        Process.setThreadPriority(Process.THREAD_PRIORITY_DISPLAY)
        var sendingThread: Thread? = null
        try {
            try {
                open()
                connectionStatus.set(STATE_CONNECTED)
                if (ipConnectionListener != null) ipConnectionListener!!.onIpConnected()
            } catch (e: IOException) {
                Timber.e("Unable to open ip connection.", e)
                return@Runnable
            }

            if (!isSendingDisabled) {
                //Launch the packet dispatching thread
                sendingThread = Thread(sendingTask, "IP Connection-Sending Thread")
                sendingThread.start()
            }

            if (!isReadingDisabled) {
                try {
                    while (connectionStatus.get() == STATE_CONNECTED) {
                        readBuffer.clear()
                        try {
                            val packetSize = read(readBuffer)
                            if (packetSize > 0) {
                                readBuffer.limit(packetSize)
                                if (ipConnectionListener != null) {
                                    readBuffer.rewind()
                                    ipConnectionListener!!.onPacketReceived(readBuffer)
                                }
                            }
                        } catch (e: InterruptedIOException) {
                            if (!isPolling) throw e
                        }
                    }
                } catch (e: IOException) {
                    Timber.e("Error occurred while reading from the connection.", e)
                }
            } else if (sendingThread != null) {
                try {
                    sendingThread.join()
                } catch (e: InterruptedException) {
                    Timber.e("Error while waiting for sending thread to complete.", e)
                }
            }
        } finally {
            if (sendingThread != null && sendingThread.isAlive) sendingThread.interrupt()
            disconnect()
            Timber.d("Exiting connection manager thread.")
        }
    }

    /**
     * Blocks until there's packet(s) to send, then dispatch them.
     */
    private val sendingTask: Runnable = object : Runnable {
        override fun run() {
            try {
                while (connectionStatus.get() == STATE_CONNECTED) {
                    val packetData = packetsToSend.take()
                    val listener = packetData.listener
                    try {
                        send(packetData)
                        postSendSuccess(listener)
                    } catch (e: IOException) {
                        Timber.e("Error occurred while sending packet.", e)
                        postSendTimeout(listener)
                    }
                }
            } catch (e: InterruptedException) {
                Timber.e("Dispatching thread was interrupted.", e)
            } finally {
                disconnect()
                Timber.d("Exiting packet dispatcher thread.")
            }
        }

        private fun postSendSuccess(listener: ICommandListener?) {
            if (handler == null || listener == null) return
            handler.post(Runnable {
                try {
                    listener.onSuccess()
                } catch (e: RemoteException) {
                    Timber.e(e.message, e)
                }
            })
        }

        private fun postSendTimeout(listener: ICommandListener?) {
            if (handler == null || listener == null) return
            handler.post(Runnable {
                try {
                    listener.onTimeout()
                } catch (e: RemoteException) {
                    Timber.e(e.message, e)
                }
            })
        }
    }

    private val isPolling: Boolean = polling
    private var managerThread: Thread? = null

    constructor(handler: Handler?, readBufferSize: Int, isPolling: Boolean) : this(handler, readBufferSize, false, false, isPolling) {}

    @JvmOverloads
    constructor(handler: Handler?, disableSending: Boolean = false, disableReading: Boolean = false) : this(handler, DEFAULT_READ_BUFFER_SIZE, disableSending, disableReading, false) {
    }

    @Throws(IOException::class)
    protected abstract fun open()
    @Throws(IOException::class)
    abstract fun read(buffer: ByteBuffer): Int
    @Throws(IOException::class)
    protected abstract fun send(data: PacketData)
    @Throws(IOException::class)
    protected abstract fun close()

    /**
     * Establish an ip connection. If successful, ConnectionListener#onIpConnected() is called.
     */
    fun connect() {
        if (connectionStatus.compareAndSet(STATE_DISCONNECTED, STATE_CONNECTING)) {
            Timber.d(TAG, "connect(): Starting manager thread.")
            managerThread = Thread(managerTask, "IP Connection-Manager Thread").apply {
                priority = Thread.MAX_PRIORITY
                start()
            }
        }
    }

    /**
     * Disconnect an existing ip connection. If successful, ConnectionListener#onIpDisconnected() is called.
     */
    fun disconnect() {
        if (connectionStatus.get() == STATE_DISCONNECTED || managerThread == null) {
            Timber.d("already disconnected")
            return
        }
        connectionStatus.set(STATE_DISCONNECTED)
        managerThread?.apply {
            if(isAlive && !isInterrupted) {
                interrupt()
            }
        }

        try {
            close()
        } catch (e: IOException) {
            Timber.e("Error occurred while closing ip connection.", e)
        }

        ipConnectionListener?.onIpDisconnected()
    }

    fun setIpConnectionListener(ipConnectionListener: IpConnectionListener?) {
        this.ipConnectionListener = ipConnectionListener
    }

    fun sendPacket(packet: ByteArray?, packetSize: Int, listener: ICommandListener) {
        if (packet == null || packetSize <= 0) return
        packetsToSend.offer(PacketData(packetSize, packet, listener))
    }

    fun getConnectionStatus(): Int {
        return connectionStatus.get()
    }

    protected class PacketData(val dataLength: Int, val data: ByteArray, val listener: ICommandListener)
    companion object {
        const val CONNECTION_TIMEOUT = 15 * 1000 //5 seconds

        /*
    Connection state
     */
        const val STATE_DISCONNECTED = 0
        const val STATE_CONNECTING = 1
        const val STATE_CONNECTED = 2

        /**
         * Size of the buffer used to read messages from the connection.
         */
        private const val DEFAULT_READ_BUFFER_SIZE = 4096
    }
}
