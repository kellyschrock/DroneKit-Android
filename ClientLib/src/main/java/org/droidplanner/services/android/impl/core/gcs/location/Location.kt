package org.droidplanner.services.android.impl.core.gcs.location

import com.o3dr.services.android.lib.coordinate.LatLongAlt

class Location(
    val coord: LatLongAlt?,
    val bearing: Float,
    val speed: Float,
    private val isAccurate: Boolean,
    val fixTime: Long,
    val accuracy: Float
) {
    interface LocationReceiver {
        fun onLocationUpdate(location: Location?)
        fun onLocationUnavailable()
    }

    interface LocationFinder {
        fun enableLocationUpdates(tag: String?, receiver: LocationReceiver?)
        fun disableLocationUpdates(tag: String?)
    }

    fun isAccurate(): Boolean {
        return !isInvalid && isAccurate
    }

    private val isInvalid: Boolean
        private get() = coord == null || coord.latitude == 0.0 && coord.longitude == 0.0
}
