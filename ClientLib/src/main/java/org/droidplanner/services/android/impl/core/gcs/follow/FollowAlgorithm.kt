package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.gcs.roi.ROIEstimator
import java.util.concurrent.atomic.AtomicBoolean

abstract class FollowAlgorithm(protected val droneMgr: MavLinkDroneManager, handler: Handler) {
    protected open val rOIEstimator: ROIEstimator?
    private val isFollowEnabled = AtomicBoolean(false)

    protected fun isFollowEnabled(): Boolean {
        return isFollowEnabled.get()
    }

    open fun enableFollow() {
        isFollowEnabled.set(true)
        if (rOIEstimator != null) rOIEstimator!!.enableFollow()
    }

    open fun disableFollow() {
        if (isFollowEnabled.compareAndSet(true, false)) {
            if (rOIEstimator != null) rOIEstimator!!.disableFollow()
        }
    }

    open fun updateAlgorithmParams(paramsMap: Map<String, Any?>) {}

    protected open fun initROIEstimator(drone: MavLinkDrone, handler: Handler): ROIEstimator? {
        return ROIEstimator(drone, handler)
    }

    fun onLocationReceived(location: Location?) {
        if (isFollowEnabled.get()) {
            if (rOIEstimator != null) rOIEstimator!!.onLocationUpdate(location)
            location?.let { processNewLocation(it) }
        }
    }

    protected abstract fun processNewLocation(location: Location)
    abstract val type: FollowModes?

    open val params: Map<String, Any?>
        get() = emptyMap()

    enum class FollowModes(private val modeName: String) {
        LEASH("Leash"),
        LEAD("Lead"),
        RIGHT("Right"),
        LEFT("Left"),
        CIRCLE("Orbit"),
        ABOVE("Above"),
        SPLINE_LEASH("Vector Leash"),
        SPLINE_ABOVE("Vector Above"),
        GUIDED_SCAN("Guided Scan"),
        LOOK_AT_ME("Look At Me"),
        SOLO_SHOT("Solo Follow Shot")
        ;

        override fun toString(): String {
            return modeName
        }

        operator fun next(): FollowModes {
            return values()[(ordinal + 1) % values().size]
        }

        fun getAlgorithmType(droneMgr: MavLinkDroneManager, handler: Handler): FollowAlgorithm {
            return when (this) {
                LEASH -> FollowLeash(droneMgr, handler, 8.0)
                LEAD -> FollowLead(droneMgr, handler, 15.0)
                RIGHT -> FollowRight(droneMgr, handler, 10.0)
                LEFT -> FollowLeft(droneMgr, handler, 10.0)
                CIRCLE -> FollowCircle(droneMgr, handler, 15.0, 10.0)
                ABOVE -> FollowAbove(droneMgr, handler)
                SPLINE_LEASH -> FollowSplineLeash(droneMgr, handler, 8.0)
                SPLINE_ABOVE -> FollowSplineAbove(droneMgr, handler)
                GUIDED_SCAN -> FollowGuidedScan(droneMgr, handler)
                LOOK_AT_ME -> FollowLookAtMe(droneMgr, handler)
                SOLO_SHOT -> FollowSoloShot(droneMgr, handler)
                else -> FollowLeash(droneMgr, handler, 8.0)
            }
        }
    }

    init {
        val drone = droneMgr.drone
        rOIEstimator = initROIEstimator(drone!!, handler)
    }
}
