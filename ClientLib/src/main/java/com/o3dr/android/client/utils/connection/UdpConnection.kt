package com.o3dr.android.client.utils.connection

import android.os.Handler
import com.o3dr.android.client.utils.connection.AbstractIpConnection
import kotlin.jvm.JvmOverloads
import kotlin.Throws
import timber.log.Timber
import com.o3dr.android.client.utils.connection.AbstractIpConnection.PacketData
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer

private val TAG = UdpConnection::class.java.simpleName

/** Created by Fredia Huya-Kouadio on 2/18/15. */
class UdpConnection : AbstractIpConnection {
    private val serverPort: Int
    private val readTimeout: Int
    private var socket: DatagramSocket? = null
    private var sendPacket: DatagramPacket? = null
    private var receivePacket: DatagramPacket? = null
    private var hostPort = 0
    private var hostAddress: InetAddress? = null

    @JvmOverloads
    constructor(handler: Handler?, serverPort: Int, readBufferSize: Int, polling: Boolean = false, readTimeout: Int = 0) : super(handler, readBufferSize, polling) {
        this.serverPort = serverPort
        if (polling) {
            this.readTimeout = if (readTimeout > 0) readTimeout else 33 //millisecond
        } else {
            this.readTimeout = CONNECTION_TIMEOUT
        }
    }

    constructor(handler: Handler?, ip: String?, port: Int, readBufferSize: Int, polling: Boolean, readTimeout: Int) : this(handler, port, readBufferSize, polling, readTimeout) {
        hostAddress = InetAddress.getByName(ip)
    }

    constructor(handler: Handler?, address: String?, hostPort: Int, serverPort: Int) : super(handler, false, true) {
        this.serverPort = serverPort
        this.hostPort = hostPort
        hostAddress = InetAddress.getByName(address)
        readTimeout = CONNECTION_TIMEOUT
    }

    @Throws(IOException::class)
    override fun open() {
        Timber.d("Opening udp connection.")
        socket = if (serverPort == -1) DatagramSocket() else DatagramSocket(serverPort)
        socket?.apply {
            broadcast = true
            reuseAddress = true
            soTimeout = readTimeout
        }
    }

    @Throws(IOException::class)
    override fun read(buffer: ByteBuffer): Int {
        if (receivePacket == null) receivePacket = DatagramPacket(buffer.array(), buffer.capacity())
        socket?.receive(receivePacket)
        hostAddress = receivePacket!!.address
        hostPort = receivePacket!!.port
        return receivePacket!!.length
    }

    @Throws(IOException::class)
    override fun send(data: PacketData) {
        if (hostAddress != null) {
            if (sendPacket == null) {
                sendPacket = DatagramPacket(data.data, data.dataLength, hostAddress, hostPort)
            } else {
                sendPacket?.apply {
                    setData(data.data, 0, data.dataLength)
                    address = hostAddress
                    port = hostPort
                }
            }
            socket!!.send(sendPacket)
        } else {
            Timber.w("Still awaiting connection from remote host.")
        }
    }

    @Throws(IOException::class)
    override fun close() {
        Timber.d("Closing udp connection.")
        if (socket != null) socket!!.close()
    }
}
