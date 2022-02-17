package org.droidplanner.services.android.impl.core.drone.autopilot.apm

import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.text.TextUtils
import android.util.Log
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.ardupilotmega.*
import com.MAVLink.common.*
import com.MAVLink.enums.MAV_MOUNT_MODE
import com.MAVLink.enums.MAV_SYS_STATUS_SENSOR
import com.github.zafarkhaja.semver.Version
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.action.*
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.mission.action.MissionActions
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTransition
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTransition.TargetState
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import com.o3dr.services.android.lib.drone.property.Parameter
import com.o3dr.services.android.lib.drone.property.Parameters
import com.o3dr.services.android.lib.drone.property.VehicleMode
import com.o3dr.services.android.lib.gcs.action.CalibrationActions
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.communication.service.MAVLinkClient
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkParameters
import org.droidplanner.services.android.impl.core.MAVLink.WaypointManager
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces
import org.droidplanner.services.android.impl.core.drone.LogMessageListener
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.variables.APMHeartBeat
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.*
import org.droidplanner.services.android.impl.core.drone.variables.calibration.AccelCalibration
import org.droidplanner.services.android.impl.core.drone.variables.calibration.MagnetometerCalibrationImpl
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser
import org.droidplanner.services.android.impl.utils.CommonApiUtils
import timber.log.Timber
import java.util.*
import java.util.regex.Pattern

/**
 * Base class for the ArduPilot autopilots
 */
abstract class ArduPilot(droneId: String?, context: Context?, mavClient: MAVLinkClient,
                         handler: Handler, warningParser: AutopilotWarningParser?,
                         logListener: LogMessageListener?) : GenericMavLinkDrone(droneId, context, handler, mavClient, warningParser, logListener) {
    private val rc: RC
    override val mission: Mission
    override val guidedPoint: GuidedPoint
    override val calibrationSetup: AccelCalibration
    override val waypointManager: WaypointManager
    private val mag: Magnetometer
    override val camera: Camera
    override val magnetometerCalibration: MagnetometerCalibrationImpl
    protected var firmwareVersionNumber = Version.forIntegers(0, 0, 0)
    override fun initHeartBeat(handler: Handler): HeartBeat {
        return APMHeartBeat(this, handler)
    }

    protected fun setAltitudeGroundAndAirSpeeds(altitude: Double, groundSpeed: Double, airSpeed: Double, climb: Double) {
//        if (this.altitude.getAltitude() != altitude) {
//            this.altitude.setAltitude(altitude);
//            notifyDroneEvent(DroneInterfaces.DroneEventsType.ALTITUDE);
//        }
        if (speed.groundSpeed != groundSpeed || speed.airSpeed != airSpeed || speed.verticalSpeed != climb) {
            speed.groundSpeed = groundSpeed
            speed.airSpeed = airSpeed
            speed.verticalSpeed = climb
            notifyDroneEvent(DroneInterfaces.DroneEventsType.SPEED)
        }
    }

    protected fun setAltitudes(altitude: Double) {
        if (this.altitude.altitude != altitude) {
            this.altitude.altitude = altitude
            notifyDroneEvent(DroneInterfaces.DroneEventsType.ALTITUDE)
        }
    }

    protected fun setGroundAndAirSpeeds(groundSpeed: Double, airSpeed: Double, climb: Double) {
        if (speed.groundSpeed != groundSpeed || speed.airSpeed != airSpeed || speed.verticalSpeed != climb) {
            speed.groundSpeed = groundSpeed
            speed.airSpeed = airSpeed
            speed.verticalSpeed = climb
            notifyDroneEvent(DroneInterfaces.DroneEventsType.SPEED)
        }
    }

    override fun getAttribute(attributeType: String?): DroneAttribute? {
        if (!TextUtils.isEmpty(attributeType)) {
            when (attributeType) {
                AttributeType.MISSION -> return CommonApiUtils.getMission(this)
                AttributeType.GUIDED_STATE -> return CommonApiUtils.getGuidedState(this)
                AttributeType.MAGNETOMETER_CALIBRATION_STATUS -> return CommonApiUtils.getMagnetometerCalibrationStatus(this)
            }
        }
        return super.getAttribute(attributeType)
    }

    override fun executeAsyncAction(action: Action?, listener: ICommandListener?): Boolean {
        val type = action!!.type
        var data = action.data
        if (data == null) {
            data = Bundle()
        }
        return when (type) {
            MissionActions.ACTION_LOAD_WAYPOINTS -> {
                CommonApiUtils.loadWaypoints(this)
                true
            }
            MissionActions.ACTION_SET_MISSION -> {
                data.classLoader = com.o3dr.services.android.lib.drone.mission.Mission::class.java.classLoader
                (data.getParcelable(MissionActions.EXTRA_MISSION) as? com.o3dr.services.android.lib.drone.mission.Mission)?.let { mission ->
                    val pushToDrone = data.getBoolean(MissionActions.EXTRA_PUSH_TO_DRONE)
                    CommonApiUtils.setMission(this, mission, pushToDrone)
                }
                true
            }
            MissionActions.ACTION_START_MISSION -> {
                val forceModeChange = data.getBoolean(MissionActions.EXTRA_FORCE_MODE_CHANGE)
                val forceArm = data.getBoolean(MissionActions.EXTRA_FORCE_ARM)
                CommonApiUtils.startMission(this, forceModeChange, forceArm, listener)
                true
            }
            ExperimentalActions.ACTION_EPM_COMMAND -> {
                val release = data.getBoolean(ExperimentalActions.EXTRA_EPM_RELEASE)
                CommonApiUtils.epmCommand(this, release, listener)
                true
            }
            ExperimentalActions.ACTION_TRIGGER_CAMERA -> {
                CommonApiUtils.triggerCamera(this)
                true
            }
            ExperimentalActions.ACTION_SET_ROI -> {
                data.getParcelable<LatLongAlt>(ExperimentalActions.EXTRA_SET_ROI_LAT_LONG_ALT)?.let { roi ->
                    MavLinkDoCmds.setROI(this, roi, listener)
                }
                true
            }
            ExperimentalActions.ACTION_SET_RELAY -> {
                val relayNumber = data.getInt(ExperimentalActions.EXTRA_RELAY_NUMBER)
                val isOn = data.getBoolean(ExperimentalActions.EXTRA_IS_RELAY_ON)
                MavLinkDoCmds.setRelay(this, relayNumber, isOn, listener)
                true
            }
            ExperimentalActions.ACTION_SET_SERVO -> {
                val channel = data.getInt(ExperimentalActions.EXTRA_SERVO_CHANNEL)
                val pwm = data.getInt(ExperimentalActions.EXTRA_SERVO_PWM)
                MavLinkDoCmds.setServo(this, channel, pwm, listener)
                true
            }
            ControlActions.ACTION_SEND_GUIDED_POINT -> {
                data.classLoader = LatLongAlt::class.java.classLoader
                val force = data.getBoolean(ControlActions.EXTRA_FORCE_GUIDED_POINT)
                data.getParcelable<LatLongAlt?>(ControlActions.EXTRA_GUIDED_POINT)?.let { guidedPoint ->
                    Timber.d("ACTION_SEND_GUIDED_POINT: guidedPoint=%s force=%s", guidedPoint, force)
                    CommonApiUtils.sendGuidedPoint(this, guidedPoint, force, listener)
                }
                true
            }
            ControlActions.ACTION_SEND_GUIDED_POINT_DIRECT -> {
                data.classLoader = LatLongAlt::class.java.classLoader
                data.getParcelable<LatLongAlt?>(ControlActions.EXTRA_GUIDED_POINT)?.let { point ->
                    Timber.d("ACTION_SEND_GUIDED_POINT_DIRECT: point=%s", point)
                    MavLinkCommands.sendGuidedPosition(this,
                            point.latitude,
                            point.longitude,
                            point.altitude)
                }
                true
            }
            ControlActions.ACTION_LOOK_AT_TARGET -> {
                val force = data.getBoolean(ControlActions.EXTRA_FORCE_GUIDED_POINT)
                data.getParcelable<LatLongAlt?>(ControlActions.EXTRA_LOOK_AT_TARGET)?.let { target ->
                    CommonApiUtils.sendLookAtTarget(this, target, force, listener)
                }

                true
            }
            ControlActions.ACTION_RESET_ROI -> {
                CommonApiUtils.sendResetROI(this, listener)
                true
            }
            ControlActions.ACTION_SET_GUIDED_ALTITUDE -> {
                val guidedAltitude = data.getDouble(ControlActions.EXTRA_ALTITUDE)
                Timber.d("ACTION_SET_GUIDED_ALTITUDE: alt=%.1f", guidedAltitude)
                CommonApiUtils.setGuidedAltitude(this, guidedAltitude)
                true
            }
            ControlActions.ACTION_VTOL_TRANSITION -> {

                val state = VTOLTransition.TargetState.fromOrdinal(data.getInt(ControlActions.EXTRA_VTOL_TARGET_STATE))
                if (state !== TargetState.Undefined) {
                    MavLinkCommands.sendVTOLTransition(this, state, listener)
                }
                true
            }
            ParameterActions.ACTION_REFRESH_PARAMETERS -> {
                CommonApiUtils.refreshParameters(this)
                true
            }
            ParameterActions.ACTION_WRITE_PARAMETERS -> {
                data.classLoader = Parameters::class.java.classLoader

                data.getParcelable<Parameters?>(ParameterActions.EXTRA_PARAMETERS)?.let { parameters ->
                    CommonApiUtils.writeParameters(this, parameters)
                    if (!updateParametersFrom(parameters)) {
                        Timber.w("Unable to update from passed parameters")
                    }
                }
                true
            }
            StateActions.ACTION_SET_VEHICLE_HOME -> {
                data.getParcelable<LatLongAlt?>(StateActions.EXTRA_VEHICLE_HOME_LOCATION)?.let { homeLoc ->
                    MavLinkDoCmds.setVehicleHome(this, homeLoc, object : AbstractCommandListener() {
                        override fun onSuccess() {
                            CommonApiUtils.postSuccessEvent(listener)
                            requestHomeUpdate()
                        }

                        override fun onError(executionError: Int) {
                            CommonApiUtils.postErrorEvent(executionError, listener)
                            requestHomeUpdate()
                        }

                        override fun onTimeout() {
                            CommonApiUtils.postTimeoutEvent(listener)
                            requestHomeUpdate()
                        }
                    })
                } ?: run {
                    CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                }
                true
            }
            CalibrationActions.ACTION_START_IMU_CALIBRATION -> {
                CommonApiUtils.startIMUCalibration(this, listener)
                true
            }
            CalibrationActions.ACTION_SEND_IMU_CALIBRATION_ACK -> {
                val imuAck = data.getInt(CalibrationActions.EXTRA_IMU_STEP)
                CommonApiUtils.sendIMUCalibrationAck(this, imuAck)
                true
            }
            CalibrationActions.ACTION_START_MAGNETOMETER_CALIBRATION -> {
                val retryOnFailure = data.getBoolean(CalibrationActions.EXTRA_RETRY_ON_FAILURE, false)
                val saveAutomatically = data.getBoolean(CalibrationActions.EXTRA_SAVE_AUTOMATICALLY, true)
                val startDelay = data.getInt(CalibrationActions.EXTRA_START_DELAY, 0)
                CommonApiUtils.startMagnetometerCalibration(this, retryOnFailure, saveAutomatically, startDelay)
                true
            }
            CalibrationActions.ACTION_CANCEL_MAGNETOMETER_CALIBRATION -> {
                CommonApiUtils.cancelMagnetometerCalibration(this)
                true
            }
            CalibrationActions.ACTION_ACCEPT_MAGNETOMETER_CALIBRATION -> {
                CommonApiUtils.acceptMagnetometerCalibration(this)
                true
            }
            GimbalActions.ACTION_SET_GIMBAL_ORIENTATION -> {
                val pitch = data.getFloat(GimbalActions.GIMBAL_PITCH)
                val roll = data.getFloat(GimbalActions.GIMBAL_ROLL)
                val yaw = data.getFloat(GimbalActions.GIMBAL_YAW)
                MavLinkDoCmds.setGimbalOrientation(this, pitch, roll, yaw, listener)
                true
            }
            GimbalActions.ACTION_RESET_GIMBAL_MOUNT_MODE, GimbalActions.ACTION_SET_GIMBAL_MOUNT_MODE -> {
                val mountMode = data.getInt(GimbalActions.GIMBAL_MOUNT_MODE, MAV_MOUNT_MODE.MAV_MOUNT_MODE_RC_TARGETING)
                Timber.i("Setting gimbal mount mode: %d", mountMode)
                val mountParam = parameterManager?.getParameter("MNT_MODE")
                if (mountParam == null) {
                    val msg = msg_mount_configure()
                    msg.target_system = sysid
                    msg.target_component = compid
                    msg.mount_mode = mountMode.toShort()
                    msg.stab_pitch = 0
                    msg.stab_roll = 0
                    msg.stab_yaw = 0
                    mavClient?.sendMessage(msg, listener)
                } else {
                    MavLinkParameters.sendParameter(this, "MNT_MODE", 1, mountMode.toFloat())
                }
                true
            }
            else -> super.executeAsyncAction(action, listener)
        }
    }

    override fun enableManualControl(data: Bundle, listener: ICommandListener?): Boolean {
        CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
        return true
    }

    override fun performArming(data: Bundle, listener: ICommandListener?): Boolean {
        val doArm = data.getBoolean(StateActions.EXTRA_ARM)
        val emergencyDisarm = data.getBoolean(StateActions.EXTRA_EMERGENCY_DISARM)
        CommonApiUtils.arm(this, doArm, emergencyDisarm, listener)
        return true
    }

    override fun setVehicleMode(data: Bundle, listener: ICommandListener?): Boolean {
        data.classLoader = VehicleMode::class.java.classLoader
        data.getParcelable<VehicleMode?>(StateActions.EXTRA_VEHICLE_MODE)?.let { mode ->
            CommonApiUtils.changeVehicleMode(this, mode, listener)
        }

        return true
    }

    override fun setVelocity(data: Bundle, listener: ICommandListener?): Boolean {
        CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
        return true
    }

    override fun performTakeoff(data: Bundle, listener: ICommandListener?): Boolean {
        val takeoffAltitude = data.getDouble(ControlActions.EXTRA_ALTITUDE)
        CommonApiUtils.doGuidedTakeoff(this, takeoffAltitude, listener)
        return true
    }

    override fun onMavLinkMessageReceived(message: MAVLinkMessage?) {
        val compId = message!!.compid
        if (compId != AUTOPILOT_COMPONENT_ID && compId != ARTOO_COMPONENT_ID && compId != TELEMETRY_RADIO_COMPONENT_ID) {
            return
        }

        if (false == parameterManager?.processMessage(message)) {
            waypointManager.processMessage(message)
            calibrationSetup.processMessage(message)
            when (message.msgid) {
                msg_statustext.MAVLINK_MSG_ID_STATUSTEXT -> {
                    // These are any warnings sent from APM:Copter with
                    // gcs_send_text_P()
                    // This includes important thing like arm fails, prearm fails, low
                    // battery, etc.
                    // also less important things like "erasing logs" and
                    // "calibrating barometer"
                    val msg_statustext = message as msg_statustext?
                    processStatusText(msg_statustext)
                }
                msg_vfr_hud.MAVLINK_MSG_ID_VFR_HUD -> processVfrHud(message as msg_vfr_hud?)
                msg_raw_imu.MAVLINK_MSG_ID_RAW_IMU -> {
                    val msg_imu = message as msg_raw_imu?
                    mag.newData(msg_imu)
                }
                msg_radio.MAVLINK_MSG_ID_RADIO -> {
                    val m_radio = message as msg_radio?
                    processSignalUpdate(m_radio!!.rxerrors, m_radio.fixed, m_radio.rssi,
                            m_radio.remrssi, m_radio.txbuf, m_radio.noise, m_radio.remnoise)
                }
                msg_rc_channels_raw.MAVLINK_MSG_ID_RC_CHANNELS_RAW -> rc.setRcInputValues(message as msg_rc_channels_raw?)
                msg_servo_output_raw.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW -> rc.setRcOutputValues(message as msg_servo_output_raw?)
                msg_camera_feedback.MAVLINK_MSG_ID_CAMERA_FEEDBACK -> camera.newImageLocation(message as msg_camera_feedback)
                msg_mount_status.MAVLINK_MSG_ID_MOUNT_STATUS -> processMountStatus(message as msg_mount_status?)
                msg_named_value_int.MAVLINK_MSG_ID_NAMED_VALUE_INT -> processNamedValueInt(message as msg_named_value_int?)
                msg_mag_cal_progress.MAVLINK_MSG_ID_MAG_CAL_PROGRESS, msg_mag_cal_report.MAVLINK_MSG_ID_MAG_CAL_REPORT -> magnetometerCalibration.processCalibrationMessage(message)
                else -> {}
            }
        }
        super.onMavLinkMessageReceived(message)
    }

    override fun processSysStatus(m_sys: msg_sys_status) {
        super.processSysStatus(m_sys)
        checkControlSensorsHealth(m_sys)
    }

    override fun setFirmwareVersion(message: String) {
        super.setFirmwareVersion(message)
        setFirmwareVersionNumber(message)
    }

    private fun setFirmwareVersionNumber(message: String) {
        firmwareVersionNumber = extractVersionNumber(message)
    }

    private fun checkControlSensorsHealth(sysStatus: msg_sys_status) {
        val isRCFailsafe = sysStatus.onboard_control_sensors_health and MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER.toLong() == 0L
        if (isRCFailsafe) {
            state?.parseAutopilotError("RC FAILSAFE")
        }
    }

    //    protected void processGlobalPositionInt(msg_global_position_int msg) {
    //        if(msg == null) return;
    //
    //        final double meters = msg.relative_alt / 1000;
    //
    //        if (this.altitude.getAltitude() != meters) {
    //            this.altitude.setAltitude(meters);
    //            notifyDroneEvent(DroneInterfaces.DroneEventsType.ALTITUDE);
    //        }
    //    }
    protected open fun processVfrHud(vfrHud: msg_vfr_hud?) {
        if (vfrHud == null) return
        setAltitudeGroundAndAirSpeeds(vfrHud.alt.toDouble(), vfrHud.groundspeed.toDouble(), vfrHud.airspeed.toDouble(), vfrHud.climb.toDouble())
    }

    protected fun processMountStatus(mountStatus: msg_mount_status?) {
        mountStatus?.let { camera.updateMountOrientation(it) }

        val eventInfo = Bundle(3)
        eventInfo.putFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_PITCH, mountStatus!!.pointing_a / 100f)
        eventInfo.putFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_ROLL, mountStatus.pointing_b / 100f)
        eventInfo.putFloat(AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_YAW, mountStatus.pointing_c / 100f)
        notifyAttributeListener(AttributeEvent.GIMBAL_ORIENTATION_UPDATED, eventInfo)
    }

    private fun processNamedValueInt(message: msg_named_value_int?) {
        if (message == null) return
        when (message.getName()) {
            "ARMMASK" -> {
                //Give information about the vehicle's ability to arm successfully.
                state?.getVehicleMode()?.let { vehicleMode ->
                    if (ApmModes.isCopter(vehicleMode.type)) {
                        val value = message.value
                        val isReadyToArm = value and (1 shl vehicleMode.number.toInt()) != 0
                        val armReadinessMsg = if (isReadyToArm) "READY TO ARM" else "UNREADY FOR ARMING"
                        logMessage(Log.INFO, armReadinessMsg)
                    }
                }
            }
        }
    }

    protected open fun processStatusText(statusText: msg_statustext?) {
        val message = statusText!!.getText()
        if (TextUtils.isEmpty(message)) return
        if (message.startsWith("ArduCopter") || message.startsWith("ArduPlane")
                || message.startsWith("ArduRover") || message.startsWith("Solo")
                || message.startsWith("APM:Copter") || message.startsWith("APM:Plane")
                || message.startsWith("APM:Rover")) {
            setFirmwareVersion(message)
        } else {

            //Try parsing as an error.
            if (false == state?.parseAutopilotError(message)) {

                //Relay to the connected client.
                var logLevel = when (statusText.severity.toInt()) {
                    APMConstants.Severity.SEVERITY_CRITICAL -> Log.ERROR
                    APMConstants.Severity.SEVERITY_HIGH -> Log.WARN
                    APMConstants.Severity.SEVERITY_MEDIUM -> Log.INFO
                    APMConstants.Severity.SEVERITY_LOW -> Log.VERBOSE
                    APMConstants.Severity.SEVERITY_USER_RESPONSE -> Log.DEBUG
                    else -> Log.VERBOSE
                }

                if (message.lowercase(Locale.getDefault()).startsWith("prearm:")) {
                    logLevel = Log.ERROR
                }
                logMessage(logLevel, message)
            }
        }
    }

    fun getBattDischarge(battRemain: Double): Double? {
        val battCap: Parameter? = parameterManager?.getParameter("BATT_CAPACITY")
        return if (battCap == null || battRemain == -1.0) {
            null
        } else (1 - battRemain / 100.0) * battCap.value
    }

    override fun processBatteryUpdate(voltage: Double, remain: Double, current: Double) {
        if (battery.batteryRemain != remain) {
            battery.batteryDischarge = getBattDischarge(remain)
        }
        super.processBatteryUpdate(voltage, remain, current)
    }

    companion object {
        val TAG = ArduPilot::class.java.simpleName
        const val AUTOPILOT_COMPONENT_ID = 1
        const val ARTOO_COMPONENT_ID = 0
        const val TELEMETRY_RADIO_COMPONENT_ID = 68
        const val FIRMWARE_VERSION_NUMBER_REGEX = "\\d+(\\.\\d{1,2})?"
        @JvmStatic
        fun extractVersionNumber(firmwareVersion: String?): Version {
            Timber.d("extractVersionNumber(): firmwareVersion=%s", firmwareVersion)
            var version = Version.forIntegers(0, 0, 0)
            val pattern = Pattern.compile(FIRMWARE_VERSION_NUMBER_REGEX)
            val matcher = pattern.matcher(firmwareVersion)
            if (matcher.find()) {
                val versionNumber = matcher.group(0) + ".0" // Adding a default patch version number for successful parsing.
                try {
                    version = Version.valueOf(versionNumber)
                } catch (e: Exception) {
                    Timber.e(e, "Firmware version invalid")
                }
            }
            return version
        }
    }

    init {
        waypointManager = WaypointManager(this, handler)
        rc = RC(this)
        mission = Mission(this)
        guidedPoint = GuidedPoint(this, handler)
        calibrationSetup = AccelCalibration(this, handler)
        magnetometerCalibration = MagnetometerCalibrationImpl(this)
        mag = Magnetometer(this)
        camera = Camera(this)
    }
}
