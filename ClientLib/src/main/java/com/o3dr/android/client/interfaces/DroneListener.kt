package com.o3dr.android.client.interfaces

import android.os.Bundle

/**
 * Created by fhuya on 11/18/14.
 */
interface DroneListener {
    fun onDroneEvent(event: String?, extras: Bundle?)
    fun onDroneServiceInterrupted(errorMsg: String)
}
