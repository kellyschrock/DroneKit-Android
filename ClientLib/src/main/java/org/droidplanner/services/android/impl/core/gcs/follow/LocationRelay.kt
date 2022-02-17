package org.droidplanner.services.android.impl.core.gcs.follow

import android.location.Location
import timber.log.Timber
import com.o3dr.services.android.lib.coordinate.LatLongAlt

private val TAG = LocationRelay::class.java.simpleName

/**
 * Created by kellys on 2/24/16.
 */
class LocationRelay {
    private var myLastLocation: Location? = null
    private var totalSpeed = 0f
    private var speedReadingCount = 0

    fun onFollowStart() {
        totalSpeed = 0f
        speedReadingCount = 0
        myLastLocation = null
    }

    /**
     * Convert the specified Android location to a local Location, and track speed/accuracy
     */
    fun toGcsLocation(androidLocation: Location?): org.droidplanner.services.android.impl.core.gcs.location.Location? {
        if (androidLocation == null) return null
        var gcsLocation: org.droidplanner.services.android.impl.core.gcs.location.Location? = null
        if (VERBOSE) Timber.d("toGcsLocation(): followLoc=$androidLocation")

        if (androidLocation.time <= 0) {
            androidLocation.time = System.currentTimeMillis()
        }

        val ok = androidLocation.hasAccuracy() && androidLocation.time > 0

        if (!ok) {
            Timber.w("toGcsLocation(): Location needs accuracy and time")
        } else {
            var distanceToLast = -1.0f
            var timeSinceLast = -1L
            val androidLocationTime = androidLocation.time
            if (myLastLocation != null) {
                distanceToLast = androidLocation.distanceTo(myLastLocation)
                timeSinceLast = androidLocationTime - myLastLocation!!.time
            }

            // mm/ms
            val currentSpeed =
                if (distanceToLast > 0f && timeSinceLast > 0) distanceToLast * 1000 / timeSinceLast else 0f
            val isAccurate = isLocationAccurate(androidLocation.accuracy, currentSpeed)
            if (VERBOSE) {
                Timber.d(
                    "toGcsLocation(): distancetoLast=%.2f timeToLast=%d currSpeed=%.2f accurate=%s",
                    distanceToLast, timeSinceLast, currentSpeed, isAccurate
                )
            }

            // Make a new location
            gcsLocation = org.droidplanner.services.android.impl.core.gcs.location.Location(
                LatLongAlt(
                    androidLocation.latitude,
                    androidLocation.longitude,
                    androidLocation.altitude
                ),
                androidLocation.bearing,
                androidLocation.speed,
                isAccurate,
                androidLocation.time,
                androidLocation.accuracy
            )
            myLastLocation = androidLocation
            if (VERBOSE) Timber.d("External location lat/lng=" + toLatLongString(androidLocation))
        }
        return gcsLocation
    }

    private fun isLocationAccurate(accuracy: Float, currentSpeed: Float): Boolean {
        if (accuracy >= LOCATION_ACCURACY_THRESHOLD) {
            Timber.w("isLocationAccurate() -- High/bad accuracy: $accuracy")
            return false
        }
        totalSpeed += currentSpeed
        val avg = totalSpeed / ++speedReadingCount

        // If moving:
        if (currentSpeed > 0) {
            // if average indicates some movement
            if (avg >= 1.0) {
                // Reject unreasonable updates.
                if (currentSpeed >= avg * JUMP_FACTOR) {
                    Timber.w("isLocationAccurate() -- High current speed: $currentSpeed")
                    return false
                }
            }
        }
        return true
    }

    companion object {
        private const val LOCATION_ACCURACY_THRESHOLD = 15.0f
        private const val JUMP_FACTOR = 4.0f
        private const val VERBOSE = true

        @JvmStatic
        fun getLatLongFromLocation(location: Location): String {
            return Location.convert(location.latitude, Location.FORMAT_DEGREES) + " " +
                    Location.convert(location.longitude, Location.FORMAT_DEGREES)
        }

        @JvmStatic
        fun toLatLongString(location: Location?): String? {
            return if (location != null) String.format(
                "%.6f, %.6f",
                location.latitude,
                location.longitude
            ) else null
        }
    }
}
