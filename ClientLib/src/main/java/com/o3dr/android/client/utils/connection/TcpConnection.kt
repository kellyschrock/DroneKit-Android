package com.o3dr.android.client.utils.connection

import android.os.Handler
import java.io.BufferedInputStream
import java.io.BufferedOutputStream
import java.io.IOException
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.ByteBuffer

/** Created by Fredia Huya-Kouadio on 2/18/15. */
class TcpConnection(handler: Handler?, private val serverIp: String, private val serverPort: Int) : AbstractIpConnection(handler) {
    private var socket: Socket? = null
    private var connOut: BufferedOutputStream? = null
    private var connIn: BufferedInputStream? = null
    @Throws(IOException::class)
    override fun open() {
        val serverAddr = InetAddress.getByName(serverIp)

        socket = Socket().apply {
            reuseAddress = true
            connect(InetSocketAddress(serverAddr, serverPort), CONNECTION_TIMEOUT)
            connOut = BufferedOutputStream(this.getOutputStream())
            connIn = BufferedInputStream(this.getInputStream())
        }
    }

    @Throws(IOException::class)
    override fun read(buffer: ByteBuffer): Int {
        return connIn?.read(buffer.array()) ?: -1
    }

    @Throws(IOException::class)
    override fun send(data: PacketData) {
        connOut?.apply {
            write(data.data, 0, data.dataLength)
            flush()
        }
    }

    @Throws(IOException::class)
    override fun close() {
        socket?.close()
    }
}
