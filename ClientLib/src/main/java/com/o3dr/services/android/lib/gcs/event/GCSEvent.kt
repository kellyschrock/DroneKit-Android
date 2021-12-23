package com.o3dr.services.android.lib.gcs.event

import com.o3dr.services.android.lib.gcs.event.GCSEvent

/**
 * Stores the list of gcs events (as action), and their extra parameters.
 * The defined events are used in system broadcasts.
 */
object GCSEvent {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.gcs.event"

    /**
     * Key to retrieve the app id for the client that caused the event.
     */
    const val EXTRA_APP_ID = PACKAGE_NAME + ".extra.APP_ID"

    /**
     * Broadcast action: a connection with a vehicle was established.
     */
    const val ACTION_VEHICLE_CONNECTION = PACKAGE_NAME + ".action.VEHICLE_CONNECTION"

    /**
     * Key to retrieve the parameter for the connection.
     */
    const val EXTRA_VEHICLE_CONNECTION_PARAMETER = PACKAGE_NAME + ".extra" +
            ".VEHICLE_CONNECTION_PARAMETER"

    /**
     * Broadcast action: the connection with the vehicle was broken.
     */
    const val ACTION_VEHICLE_DISCONNECTION = PACKAGE_NAME + ".action.VEHICLE_DISCONNECTION"
}
