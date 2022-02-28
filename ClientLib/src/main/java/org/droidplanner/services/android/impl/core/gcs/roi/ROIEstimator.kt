package org.droidplanner.services.android.impl.core.gcs.roi

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds.resetROI
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds.setROI
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.gcs.location.Location.LocationReceiver
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.newCoordFromBearingAndDistance
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Uses location data from Android's FusedLocation LocationManager at 1Hz and
 * calculates new points at 10Hz based on Last Location and Last Velocity.
 */
open class ROIEstimator(protected val drone: MavLinkDrone, protected var watchdog: Handler) :
    LocationReceiver {
    protected var realLocation: Location? = null
    protected var timeOfLastLocation: Long = 0
    protected var watchdogCallback = Runnable { updateROI() }
    protected val isFollowEnabled = AtomicBoolean(false)
    open fun enableFollow() {
        resetROI(drone, null)
        isFollowEnabled.set(true)
    }

    open fun disableFollow() {
        if (isFollowEnabled.compareAndSet(true, false)) {
            realLocation = null
            resetROI(drone, null)
            disableWatchdog()
        }
    }

    override fun onLocationUpdate(location: Location?) {
        if (!isFollowEnabled.get()) return
        realLocation = location
        timeOfLastLocation = System.currentTimeMillis()
        disableWatchdog()
        updateROI()
    }

    override fun onLocationUnavailable() {
        disableWatchdog()
    }

    protected fun disableWatchdog() {
        watchdog.removeCallbacks(watchdogCallback)
    }

    protected open fun updateROI() {
        if (realLocation == null) {
            return
        }
        val gcsCoord: LatLong? = realLocation!!.coord
        val bearing = realLocation!!.bearing.toDouble()
        val distanceTraveledSinceLastPoint = (realLocation!!.speed
                * (System.currentTimeMillis() - timeOfLastLocation) / 1000f).toDouble()
        val goCoord =
            newCoordFromBearingAndDistance(gcsCoord!!, bearing, distanceTraveledSinceLastPoint)
        sendUpdateROI(goCoord)
        if (realLocation!!.speed > 0) watchdog.postDelayed(watchdogCallback, updatePeriod)
    }

    protected open fun sendUpdateROI(goCoord: LatLong) {
        setROI(drone, LatLongAlt(goCoord.latitude, goCoord.longitude, 0.0), null)
    }

    fun isFollowEnabled(): Boolean {
        return isFollowEnabled.get()
    }

    protected open val updatePeriod: Long
        protected get() = TIMEOUT.toLong()

    companion object {
        private const val TIMEOUT = 100
    }
}
