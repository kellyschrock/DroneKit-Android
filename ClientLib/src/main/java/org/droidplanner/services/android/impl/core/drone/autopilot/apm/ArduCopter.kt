package org.droidplanner.services.android.impl.core.drone.autopilot.apm

import android.content.Context
import android.os.Bundle
import android.os.Handler
import com.MAVLink.Messages.MAVLinkMessage
import com.github.zafarkhaja.semver.Version
import com.o3dr.android.client.apis.CapabilityApi
import com.o3dr.services.android.lib.drone.action.ControlActions
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.property.Parameter
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.communication.model.DataLink
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces
import org.droidplanner.services.android.impl.core.drone.DroneManager
import org.droidplanner.services.android.impl.core.drone.LogMessageListener
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser
import org.droidplanner.services.android.impl.utils.CommonApiUtils
import java.util.concurrent.ConcurrentHashMap

open class ArduCopter(
        droneId: String?,
        context: Context?,
        mavClient: DataLink.DataLinkProvider<MAVLinkMessage?>?,
        handler: Handler?,
        warningParser: AutopilotWarningParser?,
        logListener: LogMessageListener?
) : ArduPilot(droneId, context, mavClient, handler, warningParser, logListener) {

    private val manualControlStateListeners: ConcurrentHashMap<String, ICommandListener> = ConcurrentHashMap<String, ICommandListener>()

    override val firmwareType: FirmwareType
        get() = FirmwareType.ARDU_COPTER

    override fun setVelocity(data: Bundle, listener: ICommandListener): Boolean {
        //Retrieve the normalized values
        val normalizedXVel: Float = data.getFloat(ControlActions.EXTRA_VELOCITY_X)
        val normalizedYVel: Float = data.getFloat(ControlActions.EXTRA_VELOCITY_Y)
        val normalizedZVel: Float = data.getFloat(ControlActions.EXTRA_VELOCITY_Z)
        val attitudeInRad = Math.toRadians(attitude.yaw)
        val cosAttitude = Math.cos(attitudeInRad)
        val sinAttitude = Math.sin(attitudeInRad)
        val projectedX = (normalizedXVel * cosAttitude).toFloat() - (normalizedYVel * sinAttitude).toFloat()
        val projectedY = (normalizedXVel * sinAttitude).toFloat() + (normalizedYVel * cosAttitude).toFloat()

        //Retrieve the speed parameters.
        val defaultSpeed = 5f //m/s

        //Retrieve the horizontal speed value
        val horizSpeedParam: Parameter? = parameterManager?.getParameter("WPNAV_SPEED")
        val horizontalSpeed = if (horizSpeedParam == null) defaultSpeed.toDouble() else horizSpeedParam.value / 100

        //Retrieve the vertical speed value.
        val vertSpeedParamName = if (normalizedZVel >= 0) "WPNAV_SPEED_UP" else "WPNAV_SPEED_DN"
        val vertSpeedParam: Parameter? = parameterManager?.getParameter(vertSpeedParamName)
        val verticalSpeed = if (vertSpeedParam == null) defaultSpeed.toDouble() else vertSpeedParam.value / 100

        MavLinkCommands.setVelocityInLocalFrame(this, (projectedX * horizontalSpeed).toFloat(),
                (projectedY * horizontalSpeed).toFloat(),
                (normalizedZVel * verticalSpeed).toFloat(),
                listener)
        return true
    }

    override fun destroy() {
        super.destroy()
        manualControlStateListeners.clear()
    }

    override fun enableManualControl(data: Bundle, listener: ICommandListener): Boolean {
        val enable: Boolean = data.getBoolean(ControlActions.EXTRA_DO_ENABLE)
        val appId: String = data.getString(DroneManager.EXTRA_CLIENT_APP_ID)

        val vehicleMode: ApmModes? = state?.mode
        if (enable) {
            if (vehicleMode == ApmModes.ROTOR_GUIDED) {
                CommonApiUtils.postSuccessEvent(listener)
            } else {
                state?.changeFlightMode(ApmModes.ROTOR_GUIDED, listener)
            }
            if (listener != null) {
                manualControlStateListeners[appId] = listener
            }
        } else {
            manualControlStateListeners.remove(appId)
            if (vehicleMode != ApmModes.ROTOR_GUIDED) {
                CommonApiUtils.postSuccessEvent(listener)
            } else {
                state?.changeFlightMode(ApmModes.ROTOR_LOITER, listener)
            }
        }
        return true
    }

    override fun notifyDroneEvent(event: DroneInterfaces.DroneEventsType?) {
        when (event) {
            DroneInterfaces.DroneEventsType.MODE -> {
                //Listen for vehicle mode updates, and update the manual control state listeners appropriately
                val currentMode: ApmModes? = state?.getMode()
                for (listener in manualControlStateListeners.values) {
                    if (currentMode == ApmModes.ROTOR_GUIDED) {
                        CommonApiUtils.postSuccessEvent(listener)
                    } else {
                        CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                    }
                }
            }
        }
        super.notifyDroneEvent(event)
    }

    override fun isFeatureSupported(featureId: String): Boolean {
        return when (featureId) {
            CapabilityApi.FeatureIds.KILL_SWITCH -> CommonApiUtils.isKillSwitchSupported(this)
            else -> super.isFeatureSupported(featureId)
        }
    }

    override fun brakeVehicle(listener: ICommandListener): Boolean {
        if (firmwareVersionNumber.greaterThanOrEqualTo(BRAKE_FEATURE_FIRMWARE_VERSION)) {
            state?.changeFlightMode(ApmModes.ROTOR_BRAKE, listener)
        } else {
            super.brakeVehicle(listener)
        }
        return true
    }

    companion object {
        private val BRAKE_FEATURE_FIRMWARE_VERSION = Version.forIntegers(3, 3, 0)
    }
}
