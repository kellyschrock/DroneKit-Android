package org.droidplanner.services.android.impl.communication.model

import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.model.ICommandListener

class DataLink {
    interface DataLinkProvider<T> {
        fun sendMessage(message: T, listener: ICommandListener?)
        val isConnected: Boolean
        fun openConnection()
        fun closeConnection()
        val receivedCount: Int
        val crcErrorCount: Int
        val lostPacketCount: Int
    }

    interface DataLinkListener<T> {
        fun notifyReceivedData(packet: T)
        fun onConnectionStatus(connectionStatus: LinkConnectionStatus?)
    }
}
