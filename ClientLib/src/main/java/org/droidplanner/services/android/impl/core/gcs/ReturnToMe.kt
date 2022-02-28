package org.droidplanner.services.android.impl.core.gcs

import android.os.Bundle
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import com.o3dr.services.android.lib.drone.property.Home
import com.o3dr.services.android.lib.gcs.returnToMe.ReturnToMeState
import com.o3dr.services.android.lib.gcs.returnToMe.ReturnToMeState.ReturnToMeStates
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.*
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.core.gcs.ReturnToMe
import org.droidplanner.services.android.impl.core.gcs.location.Location
import org.droidplanner.services.android.impl.core.gcs.location.Location.LocationFinder
import org.droidplanner.services.android.impl.core.gcs.location.Location.LocationReceiver
import org.droidplanner.services.android.impl.utils.CommonApiUtils.postErrorEvent
import org.droidplanner.services.android.impl.utils.CommonApiUtils.postSuccessEvent
import org.droidplanner.services.android.impl.utils.CommonApiUtils.postTimeoutEvent
import timber.log.Timber
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Return to me implementation.
 * If enabled, listen for user's gps location updates, and accordingly updates the vehicle RTL location.
 * Created by Fredia Huya-Kouadio on 9/21/15.
 */
class ReturnToMe(private val droneMgr: MavLinkDroneManager,
                 private val locationFinder: LocationFinder,
                 private val attributeListener: AttributeEventListener?)
    : OnDroneListener<MavLinkDrone>, LocationReceiver {

    private val isEnabled = AtomicBoolean(false)
    private val currentState: ReturnToMeState = ReturnToMeState()

    private var commandListener: ICommandListener? = null
    fun enable(listener: ICommandListener?) {
        if (isEnabled.compareAndSet(false, true)) {
            commandListener = listener
            val droneHome = home
            if (droneHome!!.isValid) {
                currentState.setOriginalHomeLocation(droneHome.coordinate)
            }

            //Enable return to me
            Timber.i("Enabling return to me.")
            locationFinder.enableLocationUpdates(TAG, this)
            updateCurrentState(ReturnToMeState.STATE_WAITING_FOR_VEHICLE_GPS)
        }
    }

    fun disable() {
        if (isEnabled.compareAndSet(true, false)) {
            //Disable return to me
            Timber.i("Disabling return to me.")
            locationFinder.disableLocationUpdates(TAG)
            currentState.setCurrentHomeLocation(null)

            //Reset the original home location
            val originalHomeLocation = currentState.getOriginalHomeLocation()
            if (originalHomeLocation != null) {
                MavLinkDoCmds.setVehicleHome(droneMgr.drone, originalHomeLocation, object : AbstractCommandListener() {
                    override fun onSuccess() {
                        Timber.i("Updated vehicle home location to %s", originalHomeLocation.toString())
                        droneMgr.drone!!.executeAsyncAction(requestHomeUpdateAction, null)
                    }

                    override fun onError(executionError: Int) {
                        Timber.e("Unable to update vehicle home location: %d", executionError)
                    }

                    override fun onTimeout() {
                        Timber.w("Vehicle home update timed out!")
                    }
                })
            }
            updateCurrentState(ReturnToMeState.STATE_IDLE)
            commandListener = null
        }
    }

    override fun onLocationUpdate(location: Location?) {
        location ?: return

        if (location.isAccurate()) {
            val home = home
            if (!home!!.isValid) {
                updateCurrentState(ReturnToMeState.STATE_WAITING_FOR_VEHICLE_GPS)
                return
            }
            val homePosition = home.coordinate

            //Calculate the displacement between the home location and the user location.
            val locationCoord = location.coord ?: return
            val results = FloatArray(3)
            android.location.Location.distanceBetween(homePosition!!.latitude, homePosition.longitude,
                    locationCoord.latitude, locationCoord.longitude, results)
            val displacement = results[0]
            if (displacement >= UPDATE_MINIMAL_DISPLACEMENT) {
                MavLinkDoCmds.setVehicleHome(droneMgr.drone,
                        LatLongAlt(locationCoord.latitude, locationCoord.longitude, homePosition.altitude),
                        object : AbstractCommandListener() {
                            override fun onSuccess() {
                                Timber.i("Updated vehicle home location to %s", locationCoord.toString())
                                droneMgr.drone!!.executeAsyncAction(requestHomeUpdateAction, null)
                                postSuccessEvent(commandListener)
                                updateCurrentState(ReturnToMeState.STATE_UPDATING_HOME)
                            }

                            override fun onError(executionError: Int) {
                                Timber.e("Unable to update vehicle home location: %d", executionError)
                                postErrorEvent(executionError, commandListener)
                                updateCurrentState(ReturnToMeState.STATE_ERROR_UPDATING_HOME)
                            }

                            override fun onTimeout() {
                                Timber.w("Vehicle home update timed out!")
                                postTimeoutEvent(commandListener)
                                updateCurrentState(ReturnToMeState.STATE_ERROR_UPDATING_HOME)
                            }
                        })
            }
        } else {
            updateCurrentState(ReturnToMeState.STATE_USER_LOCATION_INACCURATE)
        }
    }

    private val home: Home?
        private get() = droneMgr.drone!!.getAttribute(AttributeType.HOME) as Home?

    override fun onLocationUnavailable() {
        if (isEnabled.get()) {
            updateCurrentState(ReturnToMeState.STATE_USER_LOCATION_UNAVAILABLE)
            disable()
        }
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone) {
        when (event) {
            DroneEventsType.DISCONNECTED ->                 //Stops updating the vehicle RTL location
                disable()
            DroneEventsType.HOME -> if (isEnabled.get()) {
                val homeCoord = home!!.coordinate
                if (currentState.getOriginalHomeLocation() == null) currentState.setOriginalHomeLocation(homeCoord) else {
                    currentState.setCurrentHomeLocation(homeCoord)
                }
            }
        }
    }

    private fun updateCurrentState(@ReturnToMeStates state: Int) {
        currentState.state = state.toLong()
        if (attributeListener != null) {
            val eventInfo = Bundle()
            eventInfo.putInt(AttributeEventExtra.EXTRA_RETURN_TO_ME_STATE, state)
            attributeListener.onAttributeEvent(AttributeEvent.RETURN_TO_ME_STATE_UPDATE, eventInfo)
        }
    }

    val state: DroneAttribute
        get() = currentState

    companion object {
        const val UPDATE_MINIMAL_DISPLACEMENT = 5 //meters
        private val TAG = ReturnToMe::class.java.simpleName
        private val requestHomeUpdateAction = Action(MavLinkDrone.ACTION_REQUEST_HOME_UPDATE)
    }

    init {
        droneMgr.drone?.addDroneListener(this)
    }
}
