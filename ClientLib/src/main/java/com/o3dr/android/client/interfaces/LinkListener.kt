package com.o3dr.android.client.interfaces

import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus

/**
 * An interface that will update the caller with information about the link connection.
 *
 * This is passed to the [com.o3dr.android.client.Drone.connect]
 * method.
 */
interface LinkListener {
    /**
     * The callback that notifies the caller about the current state of the link connection.
     *
     * @param connectionStatus Contains information about the connection status.
     */
    fun onLinkStateUpdated(connectionStatus: LinkConnectionStatus)
}
