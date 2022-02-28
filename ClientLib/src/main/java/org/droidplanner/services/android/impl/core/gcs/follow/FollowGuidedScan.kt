package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds.setROI
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.roi.ROIEstimator
import timber.log.Timber

/**
 * Created by fhuya on 1/9/15.
 */
class FollowGuidedScan(droneMgr: MavLinkDroneManager?, handler: Handler?)
    : FollowAbove(droneMgr!!, handler!!) {

    override val type: FollowModes = FollowModes.GUIDED_SCAN

    override fun updateAlgorithmParams(params: Map<String, Any?>) {
        super.updateAlgorithmParams(params)
        val tempCoord = params[EXTRA_FOLLOW_ROI_TARGET] as LatLong?
        val target = if (tempCoord == null || tempCoord is LatLongAlt) {
            tempCoord as LatLongAlt?
        } else {
            LatLongAlt(tempCoord, defaultRoiAltitude)
        }
        rOIEstimator?.updateROITarget(target)
    }

    override fun initROIEstimator(drone: MavLinkDrone, handler: Handler): ROIEstimator? {
        return GuidedROIEstimator(drone, handler)
    }

    override val params: Map<String, Any?>
        get() {
            val params: MutableMap<String, Any?> = HashMap()
            params[EXTRA_FOLLOW_ROI_TARGET] = rOIEstimator?.roiTarget
            return params
        }

    override val rOIEstimator: GuidedROIEstimator?
        protected get() = super.rOIEstimator as GuidedROIEstimator?

    class GuidedROIEstimator(drone: MavLinkDrone, handler: Handler)
        : ROIEstimator(drone, handler) {

        var roiTarget: LatLongAlt? = null

        fun updateROITarget(roiTarget: LatLongAlt?) {
            this.roiTarget = roiTarget
            onLocationUpdate(null)
        }

        override fun updateROI() {
            if (roiTarget == null) {
                println("Cancelling ROI lock.")
                //Fallback to the default behavior
                super.updateROI()
            } else {
                Timber.d("ROI Target: " + roiTarget.toString())

                //Track the target until told otherwise.
                setROI(drone, roiTarget, null)
                watchdog.postDelayed(watchdogCallback, TIMEOUT)
            }
        }
    }

    companion object {
        private const val TIMEOUT: Long = 1000 //ms
        const val EXTRA_FOLLOW_ROI_TARGET = "extra_follow_roi_target"
        const val DEFAULT_FOLLOW_ROI_ALTITUDE = 10.0 //meters
        private const val defaultRoiAltitude = DEFAULT_FOLLOW_ROI_ALTITUDE
    }
}
