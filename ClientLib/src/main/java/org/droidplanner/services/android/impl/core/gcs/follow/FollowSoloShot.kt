package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloMessageLocation
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.ArduSolo
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.SoloComp
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.gcs.roi.ROIEstimator

/**
 * Created by Fredia Huya-Kouadio on 8/3/15.
 */
class FollowSoloShot(droneMgr: MavLinkDroneManager, handler: Handler)
    : FollowAlgorithm(droneMgr, handler) {

    private val soloComp: SoloComp
    private val locationCoord = LatLongAlt(0.0, 0.0, 0.0)
    private val locationSetter = SoloMessageLocation(locationCoord)

    override fun enableFollow() {
        super.enableFollow()
        soloComp.enableFollowDataConnection()
    }

    override fun disableFollow() {
        super.disableFollow()
        soloComp.disableFollowDataConnection()
    }

    override fun processNewLocation(location: Location) {
        if (location != null) {
            val receivedCoord = location.coord
            locationCoord.set((receivedCoord as LatLong))
            locationSetter.coordinate = locationCoord
            soloComp.updateFollowCenter(locationSetter)
        }
    }

    override val type: FollowModes = FollowModes.SOLO_SHOT

    override fun initROIEstimator(drone: MavLinkDrone, handler: Handler): ROIEstimator {
        return SoloROIEstimator(drone, handler, (drone as ArduSolo).soloComp)
    }

    protected class SoloROIEstimator(
        drone: MavLinkDrone,
        handler: Handler,
        private val soloComp: SoloComp
    ) : ROIEstimator(drone, handler) {
        private val locationCoord = LatLongAlt(0.0, 0.0, 0.0)
        private val locationSetter = SoloMessageLocation(locationCoord)
        override fun enableFollow() {
            isFollowEnabled.set(true)
        }

        override fun disableFollow() {
            if (isFollowEnabled.compareAndSet(true, false)) {
                realLocation = null
                disableWatchdog()
            }
        }

        override val updatePeriod: Long
            get() = 40L

        override fun sendUpdateROI(goCoord: LatLong) {
            locationCoord.set(goCoord)
            locationSetter.coordinate = locationCoord
            soloComp.updateFollowCenter(locationSetter)
        }
    }

    init {
        val drone = droneMgr.drone as ArduSolo?
        soloComp = drone!!.soloComp
    }
}
