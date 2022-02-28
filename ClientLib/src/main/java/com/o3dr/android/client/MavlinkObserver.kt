package com.o3dr.android.client

import com.o3dr.services.android.lib.mavlink.MavlinkMessageWrapper
import com.o3dr.services.android.lib.model.IMavlinkObserver

/**
 * Allows to register for mavlink message updates.
 */
abstract class MavlinkObserver : IMavlinkObserver.Stub() {
    abstract override fun onMavlinkMessageReceived(mavlinkMessageWrapper: MavlinkMessageWrapper)
}
