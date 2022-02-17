package org.droidplanner.services.android.impl.core.gcs.follow

import android.os.Bundle
import android.os.Handler
import com.o3dr.services.android.lib.drone.action.ControlActions
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra
import com.o3dr.services.android.lib.gcs.follow.FollowLocationSource
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.ArduSolo
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.drone.variables.GuidedPoint.Companion.changeToGuidedMode
import org.droidplanner.services.android.impl.core.drone.variables.GuidedPoint.Companion.isGuidedMode
import org.droidplanner.services.android.impl.core.gcs.follow.Follow
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.gcs.location.Location.LocationFinder
import org.droidplanner.services.android.impl.core.gcs.location.Location.LocationReceiver
import timber.log.Timber

class Follow(
    private val droneMgr: MavLinkDroneManager,
    handler: Handler,
    locationFinder: LocationFinder
) : OnDroneListener<MavLinkDrone?>, LocationReceiver {
    private var lastLocation: Location? = null
    private var mLocationSource: FollowLocationSource? = null

    /**
     * Set of return value for the 'toggleFollowMeState' method.
     */
    enum class FollowStates {
        FOLLOW_INVALID_STATE, FOLLOW_DRONE_NOT_ARMED, FOLLOW_DRONE_DISCONNECTED, FOLLOW_START, FOLLOW_RUNNING, FOLLOW_END
    }

    var state = FollowStates.FOLLOW_INVALID_STATE
        private set
    private val locationFinder: LocationFinder
    var followAlgorithm: FollowAlgorithm?
        private set
    private val mLocationRelay: LocationRelay
    fun enableFollowMe(source: FollowLocationSource) {
        if (!isEnabled) {
            val drone = droneMgr.drone
            val droneState = drone?.state
            if (droneState == null) {
                Timber.w("No drone for enableFollowMe(%s)", source)
                state = FollowStates.FOLLOW_INVALID_STATE
                return
            }
            if (droneMgr.isConnected) {
                if (droneState.isArmed()) {
                    changeToGuidedMode(drone, null)
                    state = FollowStates.FOLLOW_START
                    followAlgorithm!!.enableFollow()
                    droneMgr.onAttributeEvent(AttributeEvent.FOLLOW_START, Bundle())
                } else {
                    state = FollowStates.FOLLOW_DRONE_NOT_ARMED
                }
            } else {
                state = FollowStates.FOLLOW_DRONE_DISCONNECTED
            }
        }
        setLocationSource(source)
    }

    fun disableFollowMe() {
        Timber.i("disableFollowMe(): state=%s", state)
        followAlgorithm!!.disableFollow()
        setLocationSource(FollowLocationSource.NONE)
        lastLocation = null
        if (isEnabled) {
            state = FollowStates.FOLLOW_END
            droneMgr.onAttributeEvent(AttributeEvent.FOLLOW_STOP, Bundle())
        }
        val drone = droneMgr.drone
        // Send a brake command only on APM Follow, Solo Shot follow braking is handled by its Shot Manager onboard
        if (isGuidedMode(drone)
            && followAlgorithm!!.type != FollowAlgorithm.FollowModes.SOLO_SHOT
        ) {
            droneMgr.drone!!.executeAsyncAction(
                Action(ControlActions.ACTION_SEND_BRAKE_VEHICLE),
                null
            )
        }
    }

    val isEnabled: Boolean
        get() = state == FollowStates.FOLLOW_RUNNING || state == FollowStates.FOLLOW_START

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.MODE -> if (isEnabled && !isGuidedMode(drone)) {
                Timber.i("Follow enabled, but current mode is not guided. Disable follow")
                disableFollowMe()
            }
            DroneEventsType.HEARTBEAT_TIMEOUT, DroneEventsType.DISCONNECTED -> if (isEnabled) {
                disableFollowMe()
            }
        }
    }

    fun onFollowNewLocation(location: android.location.Location?) {
        Timber.d("onFollowNewLocation(%s)", location)
        val loc = mLocationRelay.toGcsLocation(location)
        if (loc != null && mLocationSource === FollowLocationSource.CLIENT_SPECIFIED) {
            onLocationUpdate(loc)
        }
    }

    override fun onLocationUpdate(location: Location?) {
        location ?: return
        location.coord ?: return

        Timber.d(
            "onLocationUpdate(): lat/lng=%.4f/%.4f accurate=%s",
            location.coord.latitude,
            location.coord.longitude,
            location.isAccurate()
        )

        if (location.isAccurate()) {
            state = FollowStates.FOLLOW_RUNNING
            lastLocation = location
            Timber.d("Sending location to followAlgorithm $followAlgorithm")
            followAlgorithm?.onLocationReceived(location)
        } else {
            Timber.d("Location not accurate")
            state = FollowStates.FOLLOW_START
        }

        val extras = Bundle()
        extras.putBoolean(AttributeEventExtra.EXTRA_ACCURATE, location.isAccurate())
        extras.putFloat(AttributeEventExtra.EXTRA_ACCURACY, location.accuracy)
        droneMgr.onAttributeEvent(AttributeEvent.FOLLOW_UPDATE, extras)
    }

    override fun onLocationUnavailable() {
        disableFollowMe()
    }

    fun setAlgorithm(algorithm: FollowAlgorithm) {
        Timber.i("setAlgorithm(): algo=$algorithm")
        if (followAlgorithm != null && followAlgorithm !== algorithm) {
            Timber.i("%s.disableFollow()", followAlgorithm)
            followAlgorithm!!.disableFollow()
        }
        followAlgorithm = algorithm
        if (isEnabled) {
            Timber.i("%s.enableFollow()", followAlgorithm)
            followAlgorithm!!.enableFollow()
            if (lastLocation != null) followAlgorithm!!.onLocationReceived(lastLocation)
        }
        droneMgr.onAttributeEvent(AttributeEvent.FOLLOW_UPDATE, Bundle())
    }

    private fun setLocationSource(source: FollowLocationSource) {
        if (!isEnabled) return
        if (mLocationSource !== source) {
            when (source) {
                FollowLocationSource.CLIENT_SPECIFIED -> {
                    Timber.d("Switch to client-specified locations")
                    locationFinder.disableLocationUpdates(TAG)
                    mLocationRelay.onFollowStart()
                }
                FollowLocationSource.INTERNAL -> {
                    Timber.d("Switch to internal locations")
                    locationFinder.enableLocationUpdates(TAG, this)
                }
                FollowLocationSource.NONE -> {
                    locationFinder.disableLocationUpdates(TAG)
                }
            }
            mLocationSource = source
        }
    }

    companion object {
        private val TAG = Follow::class.java.simpleName
    }

    init {
        val drone = droneMgr.drone
        drone?.addDroneListener(this)
        followAlgorithm =
            if (drone is ArduSolo) FollowAlgorithm.FollowModes.SOLO_SHOT.getAlgorithmType(
                droneMgr, handler
            ) else FollowAlgorithm.FollowModes.LEASH.getAlgorithmType(droneMgr, handler)
        this.locationFinder = locationFinder
        mLocationRelay = LocationRelay()
    }
}
