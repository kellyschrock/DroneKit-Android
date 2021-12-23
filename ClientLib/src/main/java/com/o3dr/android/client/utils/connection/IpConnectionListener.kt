package com.o3dr.android.client.utils.connection

import java.nio.ByteBuffer

/** Provides updates about the connection. */
interface IpConnectionListener {
    fun onIpConnected()
    fun onIpDisconnected()
    fun onPacketReceived(packetBuffer: ByteBuffer)
}
