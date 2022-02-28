package org.droidplanner.services.android.impl.core.drone.variables

import android.os.Handler
import android.os.RemoteException
import com.MAVLink.enums.MAV_TYPE
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.property.Altitude
import com.o3dr.services.android.lib.drone.property.Gps
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.SimpleCommandListener
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.Drone
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import timber.log.Timber

class GuidedPoint(myDrone: MavLinkDrone, private val handler: Handler) : DroneVariable<MavLinkDrone?>(myDrone), OnDroneListener<MavLinkDrone?> {
    var state = GuidedStates.UNINITIALIZED
        private set
    private var coord: LatLongAlt? = LatLongAlt(0.0, 0.0, 0.0)
    var altitude = 0.0 //altitude in meters
        private set
    private var mPostInitializationTask: Runnable? = null

    enum class GuidedStates {
        UNINITIALIZED, IDLE, ACTIVE
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.HEARTBEAT_FIRST, DroneEventsType.HEARTBEAT_RESTORED, DroneEventsType.MODE -> if (isGuidedMode(myDrone)) {
                initialize()
            } else {
                disable()
            }
            DroneEventsType.DISCONNECTED, DroneEventsType.HEARTBEAT_TIMEOUT -> disable()
            else -> {}
        }
    }

    fun pauseAtCurrentLocation(listener: ICommandListener?) {
        if (state == GuidedStates.UNINITIALIZED) {
            changeToGuidedMode(myDrone, listener)
        } else {
            val pos = gpsPosition
            pos?.let { newGuidedCoord(it) }
            state = GuidedStates.IDLE
        }
    }

    private val gpsPosition: LatLongAlt?
        private get() {
            val pos = getGpsPosition(myDrone)
            return if (pos != null) LatLongAlt(pos, altitude) else null
        }

    fun doGuidedTakeoff(alt: Double, listener: ICommandListener?) {
        myDrone ?: return

        if (Type.isCopter(myDrone!!.type)) {
            coord = gpsPosition
            altitude = alt
            state = GuidedStates.IDLE
            changeToGuidedMode(myDrone, object : SimpleCommandListener() {
                override fun onSuccess() {
                    MavLinkCommands.sendTakeoff(myDrone, alt, listener)
                    myDrone?.notifyDroneEvent(DroneEventsType.GUIDEDPOINT)
                }

                override fun onError(executionError: Int) {
                    if (listener != null) {
                        try {
                            listener.onError(executionError)
                        } catch (e: RemoteException) {
                            Timber.e(e, e.message)
                        }
                    }
                }

                override fun onTimeout() {
                    if (listener != null) {
                        try {
                            listener.onTimeout()
                        } catch (e: RemoteException) {
                            Timber.e(e, e.message)
                        }
                    }
                }
            })
        } else {
            if (listener != null) {
                handler.post {
                    try {
                        listener.onError(CommandExecutionError.COMMAND_UNSUPPORTED)
                    } catch (e: RemoteException) {
                        Timber.e(e, e.message)
                    }
                }
            }
        }
    }

    fun newGuidedCoord(coord: LatLong?) {
        changeCoord(LatLongAlt(coord, getDroneAltConstrained(myDrone)))
    }

    fun newGuidedCoord(coord: LatLongAlt) {
        changeCoord(coord)
    }

    fun newGuidedPosition(latitude: Double, longitude: Double, altitude: Double) {
        MavLinkCommands.sendGuidedPosition(myDrone, latitude, longitude, altitude)
    }

    fun newGuidedVelocity(xVel: Double, yVel: Double, zVel: Double) {
        MavLinkCommands.sendGuidedVelocity(myDrone, xVel, yVel, zVel)
    }

    fun newGuidedCoordAndVelocity(coord: LatLong?, xVel: Double, yVel: Double, zVel: Double) {
        changeCoordAndVelocity(LatLongAlt(coord, getDroneAltConstrained(myDrone)), xVel, yVel, zVel)
    }

    fun newGuidedCoordAndVelocity(coord: LatLongAlt, xVel: Double, yVel: Double, zVel: Double) {
        changeCoordAndVelocity(coord, xVel, yVel, zVel)
    }

    fun changeGuidedAltitude(alt: Double) {
        changeAlt(alt)
    }

    fun forcedGuidedCoordinate(coord: LatLongAlt, listener: ICommandListener?) {
        val droneGps = myDrone?.getAttribute(AttributeType.GPS) as Gps?
        if (false == droneGps?.has3DLock()) {
            Timber.w("no 3D lock, abort")
            postErrorEvent(handler, listener, CommandExecutionError.COMMAND_FAILED)
            return
        }
        if (isInitialized) {
            Timber.d("changeCoord(%s)", coord)
            changeCoord(coord)
            postSuccessEvent(handler, listener)
        } else {
            mPostInitializationTask = Runnable {
                Timber.d("mPostInitializationTask.run(): coord=%s", coord)
                changeCoord(coord)
            }
            Timber.d("changeToGuidedMode()")
            changeToGuidedMode(myDrone, listener)
        }
    }

    fun forcedGuidedCoordinate(coord: LatLongAlt, alt: Double, listener: ICommandListener?) {
        val droneGps = myDrone?.getAttribute(AttributeType.GPS) as Gps
        if (!droneGps.has3DLock()) {
            postErrorEvent(handler, listener, CommandExecutionError.COMMAND_FAILED)
            return
        }
        if (isInitialized) {
            changeCoord(coord)
            changeAlt(alt)
            postSuccessEvent(handler, listener)
        } else {
            mPostInitializationTask = Runnable {
                changeCoord(coord)
                changeAlt(alt)
            }
            changeToGuidedMode(myDrone, listener)
        }
    }

    private fun initialize() {
        if (state == GuidedStates.UNINITIALIZED) {
            coord = gpsPosition
            altitude = getDroneAltConstrained(myDrone)
            state = GuidedStates.IDLE
            myDrone?.notifyDroneEvent(DroneEventsType.GUIDEDPOINT)
        }
        if (mPostInitializationTask != null) {
            mPostInitializationTask!!.run()
            mPostInitializationTask = null
        }
    }

    private fun disable() {
        if (state == GuidedStates.UNINITIALIZED) return
        state = GuidedStates.UNINITIALIZED
        myDrone?.notifyDroneEvent(DroneEventsType.GUIDEDPOINT)
    }

    private fun changeAlt(alt: Double) {
        Timber.d("changeAlt(): state=%s alt=%.1f", state, alt)
        when (state) {
            GuidedStates.UNINITIALIZED -> {}
            GuidedStates.IDLE -> {
                state = GuidedStates.ACTIVE
                altitude = alt
                sendGuidedPoint()
            }
            GuidedStates.ACTIVE -> {
                altitude = alt
                sendGuidedPoint()
            }
        }
    }

    private fun changeCoord(coord: LatLongAlt) {
        Timber.d("changeCoord(): coord=%s state=%s", coord, state)
        when (state) {
            GuidedStates.UNINITIALIZED -> {}
            GuidedStates.IDLE -> {
                state = GuidedStates.ACTIVE
                this.coord = coord
                altitude = coord.altitude
                sendGuidedPoint()
            }
            GuidedStates.ACTIVE -> {
                this.coord = coord
                altitude = coord.altitude
                sendGuidedPoint()
            }
        }
    }

    private fun changeCoordAndVelocity(coord: LatLongAlt, xVel: Double, yVel: Double, zVel: Double) {
        when (state) {
            GuidedStates.UNINITIALIZED -> {}
            GuidedStates.IDLE -> {
                state = GuidedStates.ACTIVE
                this.coord = coord
                sendGuidedPointAndVelocity(xVel, yVel, zVel)
            }
            GuidedStates.ACTIVE -> {
                this.coord = coord
                sendGuidedPointAndVelocity(xVel, yVel, zVel)
            }
        }
    }

    private fun sendGuidedPointAndVelocity(xVel: Double, yVel: Double, zVel: Double) {
        if (state == GuidedStates.ACTIVE) {
            forceSendGuidedPointAndVelocity(myDrone, coord, altitude, xVel, yVel, zVel)
        }
    }

    private fun sendGuidedPoint() {
        if (state == GuidedStates.ACTIVE) {
            forceSendGuidedPoint(myDrone, coord, altitude)
        }
    }

    fun getCoord(): LatLong? {
        return coord
    }

    val isActive: Boolean
        get() = state == GuidedStates.ACTIVE
    val isIdle: Boolean
        get() = state == GuidedStates.IDLE
    val isInitialized: Boolean
        get() = state != GuidedStates.UNINITIALIZED

    companion object {
        @JvmStatic
        fun isGuidedMode(drone: MavLinkDrone?): Boolean {
            if (drone == null) return false
            val droneType = drone.type
            val droneMode = drone.state!!.getVehicleMode()
            if (Type.isCopter(droneType)) {
                return droneMode === ApmModes.ROTOR_GUIDED
            }
            if (Type.isPlane(droneType)) {
                return droneMode === ApmModes.FIXED_WING_GUIDED
            }
            return if (Type.isRover(droneType)) {
                droneMode === ApmModes.ROVER_GUIDED || droneMode === ApmModes.ROVER_HOLD
            } else false
        }

        private fun getGpsPosition(drone: Drone?): LatLong? {
            drone ?: return null

            val droneGps = drone.getAttribute(AttributeType.GPS) as Gps?
            return droneGps?.position
        }

        @JvmStatic
        fun changeToGuidedMode(drone: MavLinkDrone?, listener: ICommandListener?) {
            drone ?: return

            val droneState = drone.state
            val droneType = drone.type
            Timber.d("changeToGuidedMode(): state=%s droneType=%d", droneState, droneType)
            if (Type.isCopter(droneType)) {
                droneState!!.changeFlightMode(ApmModes.ROTOR_GUIDED, listener)
            } else if (Type.isPlane(droneType)) {
                //You have to send a guided point to the plane in order to trigger guided mode.
                forceSendGuidedPoint(drone, getGpsPosition(drone), getDroneAltConstrained(drone))
            } else if (Type.isRover(droneType)) {
                droneState!!.changeFlightMode(ApmModes.ROVER_GUIDED, listener)
            }
        }

        fun forceSendGuidedPoint(drone: MavLinkDrone?, coord: LatLong?, altitudeInMeters: Double) {
            drone ?: return

            Timber.d("forceSendGuidedPoint(): coord=%s altitudeInMeters=%.1f", coord, altitudeInMeters)
            drone.notifyDroneEvent(DroneEventsType.GUIDEDPOINT)
            if (coord != null) {
                if (isRover(drone)) {
                    MavLinkCommands.setGuidedMode(drone, coord.latitude, coord.longitude, altitudeInMeters)
                } else {
                    MavLinkCommands.sendGuidedPosition(drone, coord.latitude, coord.longitude, altitudeInMeters)
                }
            }
        }

        fun isRover(drone: MavLinkDrone): Boolean {
            return drone.type == MAV_TYPE.MAV_TYPE_GROUND_ROVER
        }

        fun forceSendGuidedPointAndVelocity(drone: MavLinkDrone?, coord: LatLongAlt?, altitudeInMeters: Double,
                                            xVel: Double, yVel: Double, zVel: Double) {
            drone ?: return

            drone.notifyDroneEvent(DroneEventsType.GUIDEDPOINT)
            if (coord != null) {
                MavLinkCommands.sendGuidedPositionAndVelocity(drone, coord.latitude, coord.longitude, altitudeInMeters, xVel,
                        yVel, zVel)
            }
        }

        private fun getDroneAltConstrained(drone: MavLinkDrone?): Double {
            drone ?: return 0.0

            val droneAltitude = drone.getAttribute(AttributeType.ALTITUDE) as Altitude?
            val alt = Math.floor(droneAltitude!!.altitude)
            return Math.max(alt, getDefaultMinAltitude(drone).toDouble())
        }

        fun getDefaultMinAltitude(drone: MavLinkDrone): Float {
            val droneType = drone.type
            return if (Type.isCopter(droneType)) {
                2f
            } else if (Type.isPlane(droneType)) {
                15f
            } else {
                0f
            }
        }
    }

    init {
        myDrone.addDroneListener(this)
    }
}
