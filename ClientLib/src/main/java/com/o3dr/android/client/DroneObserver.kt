package com.o3dr.android.client

import com.o3dr.services.android.lib.model.IObserver
import kotlin.Throws
import android.os.Bundle
import android.os.RemoteException

/**
 * Created by fhuya on 10/29/14.
 */
internal class DroneObserver(private val drone: Drone) : IObserver.Stub() {
    @Throws(RemoteException::class)
    override fun onAttributeUpdated(attributeEvent: String, eventExtras: Bundle) {
        drone.notifyAttributeUpdated(attributeEvent, eventExtras)
    }
}
