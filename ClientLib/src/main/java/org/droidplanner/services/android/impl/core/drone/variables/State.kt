package org.droidplanner.services.android.impl.core.drone.variables

import android.os.Handler
import android.os.RemoteException
import android.os.SystemClock
import com.MAVLink.ardupilotmega.msg_ekf_status_report
import com.MAVLink.enums.EKF_STATUS_FLAGS
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes.Companion.isValid
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser
import timber.log.Timber

class State(
        myDrone: GenericMavLinkDrone?,
        private val handler: Handler,
        private val warningParser: AutopilotWarningParser
        )
: DroneVariable<GenericMavLinkDrone?>(myDrone) {

    private var ekfStatus: msg_ekf_status_report? = null
    var isEkfPositionOk = false
        private set
    var errorId: String?
        private set
    private var armed = false

    var isFlying = false
        set(newState) {
            if (newState != isFlying) {
                field = newState
                myDrone!!.notifyDroneEvent(DroneEventsType.STATE)
                if (isFlying) {
                    resetFlightStartTime()
                }
            }
        }
    private var mode = ApmModes.UNKNOWN

    // flightTimer
    // ----------------
    var flightStartTime: Long = 0
        private set
    private val watchdogCallback = Runnable { resetWarning() }
    fun isArmed(): Boolean {
        return armed
    }

    fun parseAutopilotError(errorMsg: String?): Boolean {
        val parsedError = warningParser.parseWarning(myDrone, errorMsg)
        if (parsedError == null || parsedError.trim { it <= ' ' }.isEmpty()) return false
        if (parsedError != errorId) {
            errorId = parsedError
            myDrone!!.notifyDroneEvent(DroneEventsType.AUTOPILOT_WARNING)
        }
        handler.removeCallbacks(watchdogCallback)
        handler.postDelayed(watchdogCallback, ERROR_TIMEOUT)
        return true
    }

    fun repeatWarning() {
        if (errorId == null || errorId!!.length == 0 || errorId == warningParser.defaultWarning) return
        handler.removeCallbacks(watchdogCallback)
        handler.postDelayed(watchdogCallback, ERROR_TIMEOUT)
    }

    fun setArmed(newState: Boolean) {
        if (armed != newState) {
            armed = newState
            myDrone!!.notifyDroneEvent(DroneEventsType.ARMING)
            if (newState) {
                myDrone?.waypointManager?.let { waypointManager ->
                    waypointManager.waypoints
                }
            }
        }
        checkEkfPositionState(ekfStatus)
    }

    fun getVehicleMode() = mode

    fun setVehicleMode(mode: ApmModes) {
        if (this.mode !== mode) {
            this.mode = mode
            myDrone?.notifyDroneEvent(DroneEventsType.MODE)
        }
    }

    fun changeFlightMode(mode: ApmModes, listener: ICommandListener?) {
        if (this.mode === mode) {
            if (listener != null) {
                handler.post {
                    try {
                        listener.onSuccess()
                    } catch (e: RemoteException) {
                        Timber.e(e, e.message)
                    }
                }
            }
            return
        }
        if (isValid(mode)) {
            MavLinkCommands.changeFlightMode(myDrone, mode, listener)
        } else {
            if (listener != null) {
                handler.post {
                    try {
                        listener.onError(CommandExecutionError.COMMAND_FAILED)
                    } catch (e: RemoteException) {
                        Timber.e(e, e.message)
                    }
                }
            }
        }
    }

    private fun resetWarning() {
        var defaultWarning = warningParser.defaultWarning
        if (defaultWarning == null) defaultWarning = ""
        if (defaultWarning != errorId) {
            errorId = defaultWarning
            myDrone!!.notifyDroneEvent(DroneEventsType.AUTOPILOT_WARNING)
        }
    }

    // flightTimer
    // ----------------
    private fun resetFlightStartTime() {
        flightStartTime = SystemClock.elapsedRealtime()
    }

    fun getEkfStatus(): msg_ekf_status_report? {
        return ekfStatus
    }

    fun setEkfStatus(ekfState: msg_ekf_status_report?) {
        if (ekfStatus == null || !areEkfStatusEquals(ekfStatus, ekfState)) {
            ekfStatus = ekfState
            myDrone!!.notifyDroneEvent(DroneEventsType.EKF_STATUS_UPDATE)
        }
    }

    private fun checkEkfPositionState(ekfStatus: msg_ekf_status_report?) {
        if (ekfStatus == null) return
        val flags = ekfStatus.flags
        val isOk = if (armed) flags and EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS != 0
                && flags and EKF_STATUS_FLAGS.EKF_CONST_POS_MODE == 0 else flags and EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS != 0
                || flags and EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS != 0
        if (isEkfPositionOk != isOk) {
            isEkfPositionOk = isOk
            myDrone!!.notifyDroneEvent(DroneEventsType.EKF_POSITION_STATE_UPDATE)
            if (isEkfPositionOk) {
                myDrone!!.executeAsyncAction(requestHomeUpdateAction, null)
            }
        }
    }

    companion object {
        private const val ERROR_TIMEOUT = 5000L
        private val requestHomeUpdateAction = Action(MavLinkDrone.ACTION_REQUEST_HOME_UPDATE)
        private fun areEkfStatusEquals(one: msg_ekf_status_report?, two: msg_ekf_status_report?): Boolean {
            return one === two || !(one == null || two == null) && one.toString() == two.toString()
        }
    }

    init {
        errorId = warningParser.defaultWarning
        resetFlightStartTime()
    }
}
