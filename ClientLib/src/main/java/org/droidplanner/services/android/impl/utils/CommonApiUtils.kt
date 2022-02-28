package org.droidplanner.services.android.impl.utils

import android.os.Bundle
import android.os.RemoteException
import android.text.TextUtils
import android.util.Log
import android.view.Surface
import com.MAVLink.ardupilotmega.msg_ekf_status_report
import com.MAVLink.ardupilotmega.msg_mag_cal_progress
import com.MAVLink.ardupilotmega.msg_mag_cal_report
import com.MAVLink.enums.MAG_CAL_STATUS
import com.MAVLink.enums.MAV_TYPE
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.calibration.magnetometer.MagnetometerCalibrationProgress
import com.o3dr.services.android.lib.drone.calibration.magnetometer.MagnetometerCalibrationResult
import com.o3dr.services.android.lib.drone.calibration.magnetometer.MagnetometerCalibrationStatus
import com.o3dr.services.android.lib.drone.mission.Mission
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.MissionItem.ComplexItem
import com.o3dr.services.android.lib.drone.mission.item.complex.CameraDetail
import com.o3dr.services.android.lib.drone.mission.item.complex.StructureScanner
import com.o3dr.services.android.lib.drone.mission.item.complex.Survey
import com.o3dr.services.android.lib.drone.property.*
import com.o3dr.services.android.lib.gcs.follow.FollowState
import com.o3dr.services.android.lib.gcs.follow.FollowType
import com.o3dr.services.android.lib.mavlink.MavlinkMessageWrapper
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands.sendArmMessage
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCommands.startMission
import org.droidplanner.services.android.impl.core.MAVLink.command.doCmd.MavLinkDoCmds
import org.droidplanner.services.android.impl.core.drone.autopilot.Drone
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.ArduPilot
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes
import org.droidplanner.services.android.impl.core.drone.variables.ApmModes.Companion.getMode
import org.droidplanner.services.android.impl.core.drone.variables.GuidedPoint.Companion.changeToGuidedMode
import org.droidplanner.services.android.impl.core.drone.variables.GuidedPoint.GuidedStates
import org.droidplanner.services.android.impl.core.drone.variables.Type.Companion.isCopter
import org.droidplanner.services.android.impl.core.drone.variables.calibration.MagnetometerCalibrationImpl
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.core.gcs.follow.Follow
import org.droidplanner.services.android.impl.core.gcs.follow.Follow.FollowStates
import org.droidplanner.services.android.impl.core.gcs.follow.FollowAlgorithm.FollowModes
import org.droidplanner.services.android.impl.core.mission.survey.SplineSurveyImpl
import org.droidplanner.services.android.impl.core.mission.survey.SurveyImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.StructureScannerImpl
import org.droidplanner.services.android.impl.core.survey.Footprint
import org.droidplanner.services.android.impl.utils.ProxyUtils.getCameraDetail
import org.droidplanner.services.android.impl.utils.ProxyUtils.getMissionItemImpl
import org.droidplanner.services.android.impl.utils.ProxyUtils.getProxyMissionItem
import timber.log.Timber
import java.util.*

/**
 * Created by Fredia Huya-Kouadio on 3/23/15.
 */
object CommonApiUtils {
    val TAG = CommonApiUtils::class.java.simpleName
    @JvmStatic
    fun postSuccessEvent(listener: ICommandListener?) {
        listener?.onSuccess()
    }

    @JvmStatic
    fun postErrorEvent(errorCode: Int, listener: ICommandListener?) {
        listener?.onError(errorCode)
    }

    @JvmStatic
    fun postTimeoutEvent(listener: ICommandListener?) {
        listener?.onTimeout()
    }

    @JvmStatic
    fun getVehicleMode(mode: ApmModes?): VehicleMode? {
        return when (mode) {
            ApmModes.FIXED_WING_MANUAL -> VehicleMode.PLANE_MANUAL
            ApmModes.FIXED_WING_CIRCLE -> VehicleMode.PLANE_CIRCLE
            ApmModes.FIXED_WING_STABILIZE -> VehicleMode.PLANE_STABILIZE
            ApmModes.FIXED_WING_TRAINING -> VehicleMode.PLANE_TRAINING
            ApmModes.FIXED_WING_ACRO -> VehicleMode.PLANE_ACRO
            ApmModes.FIXED_WING_FLY_BY_WIRE_A -> VehicleMode.PLANE_FLY_BY_WIRE_A
            ApmModes.FIXED_WING_FLY_BY_WIRE_B -> VehicleMode.PLANE_FLY_BY_WIRE_B
            ApmModes.FIXED_WING_CRUISE -> VehicleMode.PLANE_CRUISE
            ApmModes.FIXED_WING_AUTOTUNE -> VehicleMode.PLANE_AUTOTUNE
            ApmModes.FIXED_WING_AUTO -> VehicleMode.PLANE_AUTO
            ApmModes.FIXED_WING_RTL -> VehicleMode.PLANE_RTL
            ApmModes.FIXED_WING_LOITER -> VehicleMode.PLANE_LOITER
            ApmModes.FIXED_WING_GUIDED -> VehicleMode.PLANE_GUIDED
            ApmModes.ROTOR_STABILIZE -> VehicleMode.COPTER_STABILIZE
            ApmModes.ROTOR_ACRO -> VehicleMode.COPTER_ACRO
            ApmModes.ROTOR_ALT_HOLD -> VehicleMode.COPTER_ALT_HOLD
            ApmModes.ROTOR_AUTO -> VehicleMode.COPTER_AUTO
            ApmModes.ROTOR_GUIDED -> VehicleMode.COPTER_GUIDED
            ApmModes.ROTOR_LOITER -> VehicleMode.COPTER_LOITER
            ApmModes.ROTOR_RTL -> VehicleMode.COPTER_RTL
            ApmModes.ROTOR_CIRCLE -> VehicleMode.COPTER_CIRCLE
            ApmModes.ROTOR_LAND -> VehicleMode.COPTER_LAND
            ApmModes.ROTOR_TOY -> VehicleMode.COPTER_DRIFT
            ApmModes.ROTOR_SPORT -> VehicleMode.COPTER_SPORT
            ApmModes.ROTOR_FLIP -> VehicleMode.COPTER_FLIP
            ApmModes.ROTOR_AUTOTUNE -> VehicleMode.COPTER_AUTOTUNE
            ApmModes.ROTOR_POSHOLD -> VehicleMode.COPTER_POSHOLD
            ApmModes.ROTOR_BRAKE -> VehicleMode.COPTER_BRAKE
            ApmModes.ROTOR_THROW -> VehicleMode.COPTER_THROW
            ApmModes.ROTOR_AVOID_ADSB -> VehicleMode.COPTER_AVOID_ADSB
            ApmModes.ROTOR_GUIDED_NOGPS -> VehicleMode.COPTER_GUIDED_NOGPS
            ApmModes.ROTOR_SMART_RTL -> VehicleMode.COPTER_SMART_RTL
            ApmModes.ROVER_MANUAL -> VehicleMode.ROVER_MANUAL
            ApmModes.ROVER_STEERING -> VehicleMode.ROVER_STEERING
            ApmModes.ROVER_HOLD -> VehicleMode.ROVER_HOLD
            ApmModes.ROVER_LOITER -> VehicleMode.ROVER_LOITER
            ApmModes.ROVER_FOLLOW -> VehicleMode.ROVER_FOLLOW
            ApmModes.ROVER_SIMPLE -> VehicleMode.ROVER_SIMPLE
            ApmModes.ROVER_AUTO -> VehicleMode.ROVER_AUTO
            ApmModes.ROVER_RTL -> VehicleMode.ROVER_RTL
            ApmModes.ROVER_SMARTRTL -> VehicleMode.ROVER_SMART_RTL
            ApmModes.ROVER_ACRO -> VehicleMode.ROVER_ACRO
            ApmModes.ROVER_GUIDED -> VehicleMode.ROVER_GUIDED
            ApmModes.ROVER_INITIALIZING -> VehicleMode.ROVER_INITIALIZING
            ApmModes.VTOL_STABILIZE -> VehicleMode.VTOL_STABILIZE
            ApmModes.VTOL_HOVER -> VehicleMode.VTOL_HOVER
            ApmModes.VTOL_LOITER -> VehicleMode.VTOL_LOITER
            ApmModes.VTOL_LAND -> VehicleMode.VTOL_LAND
            ApmModes.VTOL_RTL -> VehicleMode.VTOL_RTL
            ApmModes.UNKNOWN -> null
            else -> null
        }
    }

    @JvmStatic
    fun getDroneProxyType(originalType: Int): Int {
        return when (originalType) {
            MAV_TYPE.MAV_TYPE_TRICOPTER,
            MAV_TYPE.MAV_TYPE_QUADROTOR,
            MAV_TYPE.MAV_TYPE_HEXAROTOR,
            MAV_TYPE.MAV_TYPE_OCTOROTOR,
            MAV_TYPE.MAV_TYPE_HELICOPTER -> Type.TYPE_COPTER
            MAV_TYPE.MAV_TYPE_FIXED_WING -> Type.TYPE_PLANE
            MAV_TYPE.MAV_TYPE_GROUND_ROVER,
            MAV_TYPE.MAV_TYPE_SURFACE_BOAT -> Type.TYPE_ROVER
            MAV_TYPE.MAV_TYPE_VTOL_DUOROTOR,
            MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR,
            MAV_TYPE.MAV_TYPE_VTOL_TILTROTOR,
            MAV_TYPE.MAV_TYPE_VTOL_RESERVED2,
            MAV_TYPE.MAV_TYPE_VTOL_RESERVED3,
            MAV_TYPE.MAV_TYPE_VTOL_RESERVED4,
            MAV_TYPE.MAV_TYPE_VTOL_RESERVED5 -> Type.TYPE_VTOL
            else -> -1
        }
    }

    @JvmStatic
    fun getProxyCameraFootPrint(footprint: Footprint?): FootPrint? {
        return if (footprint == null) null else FootPrint(footprint.gSD, footprint.vertexInGlobalFrame)
    }

    @JvmStatic
    fun followTypeToMode(drone: MavLinkDrone, followType: FollowType?): FollowModes {
        return when (followType) {
            FollowType.ABOVE -> if (drone.firmwareType === FirmwareType.ARDU_SOLO) FollowModes.SPLINE_ABOVE else FollowModes.ABOVE
            FollowType.LEAD -> FollowModes.LEAD
            FollowType.LEASH -> if (drone.firmwareType === FirmwareType.ARDU_SOLO) FollowModes.SPLINE_LEASH else FollowModes.LEASH
            FollowType.CIRCLE -> FollowModes.CIRCLE
            FollowType.LEFT -> FollowModes.LEFT
            FollowType.RIGHT -> FollowModes.RIGHT
            FollowType.GUIDED_SCAN -> FollowModes.GUIDED_SCAN
            FollowType.LOOK_AT_ME -> FollowModes.LOOK_AT_ME
            FollowType.SOLO_SHOT -> FollowModes.SOLO_SHOT
            else -> if (drone.firmwareType === FirmwareType.ARDU_SOLO) FollowModes.SPLINE_LEASH else FollowModes.LEASH
        }
    }

    @JvmStatic
    fun followModeToType(followMode: FollowModes?): FollowType {
        return when (followMode) {
            FollowModes.LEASH, FollowModes.SPLINE_LEASH -> FollowType.LEASH
            FollowModes.LEAD -> FollowType.LEAD
            FollowModes.RIGHT -> FollowType.RIGHT
            FollowModes.LEFT -> FollowType.LEFT
            FollowModes.CIRCLE -> FollowType.CIRCLE
            FollowModes.ABOVE, FollowModes.SPLINE_ABOVE -> FollowType.ABOVE
            FollowModes.GUIDED_SCAN -> FollowType.GUIDED_SCAN
            FollowModes.LOOK_AT_ME -> FollowType.LOOK_AT_ME
            FollowModes.SOLO_SHOT -> FollowType.SOLO_SHOT
            else -> FollowType.LEASH
        }
    }

    @JvmStatic
    fun getCameraProxy(drone: Drone, cameraDetails: List<CameraDetail>): CameraProxy {
        val camDetail: CameraDetail?
        val currentFieldOfView: FootPrint?
        val proxyPrints: MutableList<FootPrint> = ArrayList()
        if (drone !is MavLinkDrone) {
            camDetail = CameraDetail()
            currentFieldOfView = FootPrint()
        } else {
            val droneCamera = drone.camera
            camDetail = getCameraDetail(droneCamera!!.camera)
            val footprints = droneCamera.getFootprints()
            for (footprint in footprints) {
                getProxyCameraFootPrint(footprint)?.let { foot ->
                    proxyPrints.add(foot)
                }
            }
            val droneGps = drone.getAttribute(AttributeType.GPS) as Gps?
            currentFieldOfView = if (droneGps != null && droneGps.isValid) getProxyCameraFootPrint(droneCamera.currentFieldOfView) else FootPrint()
        }
        return CameraProxy(camDetail!!, currentFieldOfView!!, proxyPrints, cameraDetails)
    }

    @JvmStatic
    fun getState(drone: MavLinkDrone?, isConnected: Boolean, vibration: Vibration?, sysid: Short, compid: Short): State {
        if (drone == null) return State()
        val droneState = drone.state
        val droneMode = droneState!!.getVehicleMode()
        val accelCalibration = drone.calibrationSetup
        val calibrationMessage = if (accelCalibration != null && accelCalibration.isCalibrating) accelCalibration.message else null
        return State(isConnected, getVehicleMode(droneMode), droneState.isArmed(),
                droneState.isFlying, droneState.errorId, drone.mavlinkVersion, calibrationMessage,
                droneState.flightStartTime, generateEkfStatus(droneState.getEkfStatus()),
                isConnected && drone.isConnectionAlive, vibration, sysid, compid)
    }

    @JvmStatic
    fun generateEkfStatus(ekfStatus: msg_ekf_status_report?): EkfStatus? {
        return if (ekfStatus == null) {
            null
        } else EkfStatus(ekfStatus.flags, ekfStatus.compass_variance,
                ekfStatus.pos_horiz_variance, ekfStatus.terrain_alt_variance, ekfStatus.velocity_variance,
                ekfStatus.pos_vert_variance)
    }

    @JvmStatic
    fun getMission(drone: MavLinkDrone?): Mission {
        val proxyMission = Mission()
        if (drone == null) return proxyMission
        val droneMission = drone.mission
        val droneMissionItemImpls = droneMission!!.getComponentItems()
        proxyMission.currentMissionItem = drone.missionStats!!.currentWP.toInt()
        if (droneMissionItemImpls.isNotEmpty()) {
            for (item in droneMissionItemImpls) {
                proxyMission.addMissionItem(getProxyMissionItem(item)!!)
            }
        }
        return proxyMission
    }

    @JvmStatic
    fun getType(drone: MavLinkDrone?): Type {
        return if (drone == null) Type() else Type(getDroneProxyType(drone.type), drone.firmwareVersion)
    }

    @JvmStatic
    fun getGuidedState(drone: MavLinkDrone?): GuidedState {
        if (drone == null) return GuidedState()
        val guidedPoint = drone.guidedPoint
        val guidedState: Int
        guidedState = when (guidedPoint!!.state) {
            GuidedStates.UNINITIALIZED -> GuidedState.STATE_UNINITIALIZED
            GuidedStates.ACTIVE -> GuidedState.STATE_ACTIVE
            GuidedStates.IDLE -> GuidedState.STATE_IDLE
            else -> GuidedState.STATE_UNINITIALIZED
        }
        var guidedCoord = guidedPoint.getCoord()
        if (guidedCoord == null) {
            guidedCoord = LatLong(0.0, 0.0)
        }
        val guidedAlt = guidedPoint.altitude
        return GuidedState(guidedState, LatLongAlt(guidedCoord, guidedAlt))
    }

    @JvmStatic
    fun changeVehicleMode(drone: MavLinkDrone?, newMode: VehicleMode, listener: ICommandListener?) {
        if (drone == null) return
        val mavType: Int
        mavType = when (newMode.droneType) {
            Type.TYPE_COPTER -> MAV_TYPE.MAV_TYPE_QUADROTOR
            Type.TYPE_PLANE -> MAV_TYPE.MAV_TYPE_FIXED_WING
            Type.TYPE_ROVER -> MAV_TYPE.MAV_TYPE_GROUND_ROVER
            else -> MAV_TYPE.MAV_TYPE_QUADROTOR
        }
        drone.state!!.changeFlightMode(getMode(newMode.mode.toLong(), mavType), listener)
    }

    @JvmStatic
    fun getFollowState(followMe: Follow?): FollowState {
        if (followMe == null) return FollowState()
        val state: Int
        state = when (followMe.state) {
            FollowStates.FOLLOW_INVALID_STATE -> FollowState.STATE_INVALID
            FollowStates.FOLLOW_DRONE_NOT_ARMED -> FollowState.STATE_DRONE_NOT_ARMED
            FollowStates.FOLLOW_DRONE_DISCONNECTED -> FollowState.STATE_DRONE_DISCONNECTED
            FollowStates.FOLLOW_START -> FollowState.STATE_START
            FollowStates.FOLLOW_RUNNING -> FollowState.STATE_RUNNING
            FollowStates.FOLLOW_END -> FollowState.STATE_END
            else -> FollowState.STATE_INVALID
        }
        val currentAlg = followMe.followAlgorithm
        val params = Bundle()

        currentAlg?.params?.let { modeParams ->
            for ((key, value) in modeParams) {
                when (key) {
                    FollowType.EXTRA_FOLLOW_ROI_TARGET -> {
                        val target = value as LatLongAlt
                        if (target != null) {
                            params.putParcelable(key, target)
                        }
                    }
                    FollowType.EXTRA_FOLLOW_RADIUS -> {
                        val radius = value as Double
                        if (radius != null) params.putDouble(key, radius)
                    }
                }
            }
        }

        return FollowState(state, followModeToType(currentAlg?.type), params)
    }

    @JvmStatic
    fun disableFollowMe(follow: Follow?) {
        follow?.disableFollowMe()
    }

    @JvmStatic
    fun triggerCamera(drone: MavLinkDrone?) {
        if (drone == null) return
        MavLinkDoCmds.triggerCamera(drone)
    }

    @JvmStatic
    fun epmCommand(drone: MavLinkDrone?, release: Boolean, listener: ICommandListener?) {
        if (drone == null) return
        MavLinkDoCmds.empCommand(drone, release, listener)
    }

    @JvmStatic
    fun loadWaypoints(drone: MavLinkDrone?) {
        if (drone == null) return
        drone.waypointManager!!.waypoints
    }

    @JvmStatic
    fun refreshParameters(drone: MavLinkDrone?) {
        if (drone == null) return
        drone.parameterManager!!.refreshParameters()
    }

    @JvmStatic
    fun writeParameters(drone: MavLinkDrone?, parameters: Parameters?) {
        Timber.d("writeParameters(): params=%s", parameters)
        if (drone == null || parameters == null) return
        val parametersList = parameters.parameters
        if (parametersList.isEmpty()) {
            Timber.w("No params to write")
            return
        }
        val droneParams = drone.parameterManager
        for (proxyParam in parametersList) {
            droneParams!!.sendParameter(proxyParam)
        }
    }

    @JvmStatic
    fun setMission(drone: MavLinkDrone?, mission: Mission, pushToDrone: Boolean) {
        if (drone == null) return
        val droneMission = drone.mission
        droneMission!!.clearMissionItems()
        val itemsList: List<MissionItem> = mission.missionItems
        for (item in itemsList) {
            droneMission.addMissionItem(getMissionItemImpl(droneMission, item))
        }
        if (pushToDrone) droneMission.sendMissionToAPM()
    }

    @JvmStatic
    fun startMission(drone: ArduPilot?, forceModeChange: Boolean, forceArm: Boolean, listener: ICommandListener?) {
        if (drone == null) {
            return
        }
        val sendCommandRunnable = Runnable { startMission(drone, listener) }
        val modeCheckRunnable = Runnable {
            if (drone.state?.getVehicleMode() !== ApmModes.ROTOR_AUTO) {
                if (forceModeChange) {
                    changeVehicleMode(drone, VehicleMode.COPTER_AUTO, object : AbstractCommandListener() {
                        override fun onSuccess() {
                            sendCommandRunnable.run()
                        }

                        override fun onError(executionError: Int) {
                            postErrorEvent(executionError, listener)
                        }

                        override fun onTimeout() {
                            postTimeoutEvent(listener)
                        }
                    })
                } else {
                    postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                }
                return@Runnable
            } else {
                sendCommandRunnable.run()
            }
        }

        if (false == drone.state?.isArmed()) {
            if (forceArm) {
                arm(drone, true, object : AbstractCommandListener() {
                    override fun onSuccess() {
                        modeCheckRunnable.run()
                    }

                    override fun onError(executionError: Int) {
                        postErrorEvent(executionError, listener)
                    }

                    override fun onTimeout() {
                        postTimeoutEvent(listener)
                    }
                })
            } else {
                postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            }
            return
        }
        modeCheckRunnable.run()
    }

    @JvmStatic
    fun generateDronie(drone: MavLinkDrone?): Float {
        return if (drone == null) (-1).toFloat() else drone.mission!!.makeAndUploadDronie().toFloat()
    }

    @JvmStatic
    fun arm(drone: ArduPilot?, arm: Boolean, listener: ICommandListener?) {
        arm(drone, arm, false, listener)
    }

    @JvmStatic
    fun arm(drone: ArduPilot?, arm: Boolean, emergencyDisarm: Boolean, listener: ICommandListener?) {
        if (drone == null) return
        if (!arm && emergencyDisarm) {
            if (isCopter(drone.type) && !isKillSwitchSupported(drone)) {
                changeVehicleMode(drone, VehicleMode.COPTER_STABILIZE, object : AbstractCommandListener() {
                    override fun onSuccess() {
                        sendArmMessage(drone, arm, emergencyDisarm, listener)
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
                return
            }
        }
        sendArmMessage(drone, arm, emergencyDisarm, listener)
    }

    /**
     * Check if the kill switch feature is supported on the given drone
     *
     * @param drone
     * @return true if it's supported, false otherwise.
     */
    @JvmStatic
    fun isKillSwitchSupported(drone: MavLinkDrone?): Boolean {
        if (drone == null) return false
        if (!isCopter(drone.type)) return false
        val firmwareVersion = drone.firmwareVersion
        return if (TextUtils.isEmpty(firmwareVersion)) false else !(!firmwareVersion!!.startsWith("APM:Copter V3.3")
                && !firmwareVersion.startsWith("APM:Copter V3.4")
                && !firmwareVersion.startsWith("Solo"))
    }

    @JvmStatic
    fun startMagnetometerCalibration(drone: MavLinkDrone?, retryOnFailure: Boolean, saveAutomatically: Boolean, startDelay: Int) {
        if (drone == null) return
        drone.magnetometerCalibration!!.startCalibration(retryOnFailure, saveAutomatically, startDelay)
    }

    @JvmStatic
    fun cancelMagnetometerCalibration(drone: MavLinkDrone?) {
        if (drone == null) return
        drone.magnetometerCalibration!!.cancelCalibration()
    }

    @JvmStatic
    fun acceptMagnetometerCalibration(drone: MavLinkDrone?) {
        if (drone == null) return
        drone.magnetometerCalibration!!.acceptCalibration()
    }

    @JvmStatic
    fun startIMUCalibration(drone: MavLinkDrone?, listener: ICommandListener?) {
        if (drone != null) drone.calibrationSetup!!.startCalibration(listener)
    }

    @JvmStatic
    fun sendIMUCalibrationAck(drone: MavLinkDrone?, step: Int) {
        if (drone == null) return
        drone.calibrationSetup!!.sendAck(step)
    }

    @JvmStatic
    fun doGuidedTakeoff(drone: MavLinkDrone?, altitude: Double, listener: ICommandListener?) {
        if (drone == null) return
        drone.guidedPoint!!.doGuidedTakeoff(altitude, listener)
    }

    @JvmStatic
    fun sendMavlinkMessage(drone: MavLinkDrone?, messageWrapper: MavlinkMessageWrapper?) {
        if (drone == null || messageWrapper == null) {
            Timber.d("No drone or messageWrapper")
            return
        }
        val message = messageWrapper.mavLinkMessage
        if (message == null) {
            Timber.d("No message to send")
            return
        }
        message.compid = drone.compid.toInt()
        message.sysid = drone.sysid.toInt()
        Timber.d("compid=" + message.compid + " sysid=" + message.sysid)

        //Set the target system and target component for MAVLink messages that support those
        //attributes.
        try {
            val tempMessage: Class<*> = message.javaClass
            val target_system = tempMessage.getDeclaredField("target_system")
            val target_component = tempMessage.getDeclaredField("target_component")
            target_system.setShort(message, message.sysid.toShort())
            target_component.setShort(message, message.compid.toShort())
        } catch (e: NoSuchFieldException) {
            Log.v(TAG, String.format("No target_system/target_component fields in %s", message.javaClass.name))
        } catch (e: SecurityException) {
            Timber.e(e, e.message)
        } catch (e: IllegalAccessException) {
            Timber.e(e, e.message)
        } catch (e: IllegalArgumentException) {
            Timber.e(e, e.message)
        } catch (e: ExceptionInInitializerError) {
            Timber.e(e, e.message)
        }
        drone.mavClient!!.sendMessage(message, null)
    }

    @JvmStatic
    fun sendMavlinkMessage(drone: MavLinkDrone?, messageWrapper: MavlinkMessageWrapper?,
                           targetSysId: Short, targetComponentId: Short) {
        if (drone == null || messageWrapper == null) {
            Timber.d("No drone or messageWrapper")
            return
        }
        val message = messageWrapper.mavLinkMessage
        if (message == null) {
            Timber.d("No message to send")
            return
        }
        message.compid = drone.compid.toInt()
        message.sysid = drone.sysid.toInt()
        Timber.d("sendMavlinkMessage(): msg.sysid=%d msg.compid=%d targetSys=%d targetComp=%d",
                message.sysid, message.compid, targetSysId, targetComponentId)

        //Set the target system and target component for MAVLink messages that support those
        //attributes.
        try {
            val tempMessage: Class<*> = message.javaClass
            val target_system = tempMessage.getDeclaredField("target_system")
            val target_component = tempMessage.getDeclaredField("target_component")
            target_system.setShort(message, targetSysId)
            target_component.setShort(message, targetComponentId)
        } catch (e: NoSuchFieldException) {
            Log.v(TAG, String.format("No target_system/target_component fields in %s", message.javaClass.name))
        } catch (e: SecurityException) {
            Timber.e(e, e.message)
        } catch (e: IllegalAccessException) {
            Timber.e(e, e.message)
        } catch (e: IllegalArgumentException) {
            Timber.e(e, e.message)
        } catch (e: ExceptionInInitializerError) {
            Timber.e(e, e.message)
        }
        drone.mavClient!!.sendMessage(message, null)
    }

    @JvmStatic
    fun sendGuidedPoint(drone: MavLinkDrone?, point: LatLongAlt?, force: Boolean, listener: ICommandListener?) {
        if (drone == null) return
        val guidedPoint = drone.guidedPoint
        Timber.d("sendGuidedPoint(): point=%s guidedPoint=%s force=%s", point, guidedPoint, force)
        if (guidedPoint!!.isInitialized) {
            guidedPoint.newGuidedCoord(point!!)
        } else if (force) {
            try {
                guidedPoint.forcedGuidedCoordinate(point!!, listener)
            } catch (e: Exception) {
                Timber.e(e, e.message)
            }
        }
    }

    @JvmStatic
    fun sendResetROI(drone: MavLinkDrone?, listener: ICommandListener?) {
        if (drone == null) {
            return
        }
        MavLinkDoCmds.resetROI(drone, listener)
    }

    @JvmStatic
    fun sendLookAtTarget(drone: MavLinkDrone?, target: LatLongAlt?, force: Boolean, listener: ICommandListener?) {
        if (drone == null) return
        val guidedPoint = drone.guidedPoint
        if (guidedPoint!!.isInitialized) {
            MavLinkDoCmds.setROI(drone, target, listener)
        } else if (force) {
            changeToGuidedMode(drone, object : AbstractCommandListener() {
                override fun onSuccess() {
                    MavLinkDoCmds.setROI(drone, target, listener)
                }

                override fun onError(executionError: Int) {
                    postErrorEvent(executionError, listener)
                }

                override fun onTimeout() {
                    postTimeoutEvent(listener)
                }
            })
        }
    }

    @JvmStatic
    fun setGuidedAltitude(drone: MavLinkDrone?, altitude: Double) {
        if (drone == null) return
        drone.guidedPoint!!.changeGuidedAltitude(altitude)
    }

    @JvmStatic
    fun gotoWaypoint(drone: MavLinkDrone?, waypoint: Int, listener: ICommandListener?) {
        if (drone == null) return
        if (waypoint < 0) {
            postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            return
        }
        MavLinkDoCmds.gotoWaypoint(drone, waypoint, listener)
    }

    @JvmStatic
    fun buildComplexMissionItem(drone: MavLinkDrone?, itemBundle: Bundle?) {
        val missionItem = MissionItemType.restoreMissionItemFromBundle<MissionItem>(itemBundle)
        if (missionItem == null || missionItem !is ComplexItem<*>) return
        val itemType = missionItem.type
        when (itemType) {
            MissionItemType.SURVEY -> {
                val updatedSurvey = buildSurvey(drone, missionItem as Survey)
                if (updatedSurvey != null) itemType.storeMissionItem(updatedSurvey, itemBundle)
            }
            MissionItemType.SPLINE_SURVEY -> {
                val updatedSplineSurvey = buildSplineSurvey(drone, missionItem as Survey)
                if (updatedSplineSurvey != null) itemType.storeMissionItem(updatedSplineSurvey, itemBundle)
            }
            MissionItemType.STRUCTURE_SCANNER -> {
                val updatedScanner = buildStructureScanner(drone, missionItem as StructureScanner)
                if (updatedScanner != null) itemType.storeMissionItem(updatedScanner, itemBundle)
            }
            else -> Timber.w("Unrecognized complex mission item.")
        }
    }

    @JvmStatic
    fun buildSurvey(drone: MavLinkDrone?, survey: Survey?): Survey? {
        val droneMission = drone?.mission
        val updatedSurveyImpl = getMissionItemImpl(droneMission, survey) as SurveyImpl?
        return getProxyMissionItem(updatedSurveyImpl) as Survey?
    }

    fun buildSplineSurvey(drone: MavLinkDrone?, survey: Survey?): Survey? {
        val droneMission = drone?.mission
        val updatedSplineSurvey = getMissionItemImpl(droneMission, survey) as SplineSurveyImpl?
        return getProxyMissionItem(updatedSplineSurvey) as Survey?
    }

    @JvmStatic
    fun buildStructureScanner(drone: MavLinkDrone?, item: StructureScanner?): StructureScanner? {
        val droneMission = drone?.mission
        val updatedScan = getMissionItemImpl(droneMission, item) as StructureScannerImpl?
        return getProxyMissionItem(updatedScan) as StructureScanner?
    }

    @JvmStatic
    fun getMagnetometerCalibrationStatus(drone: MavLinkDrone?): MagnetometerCalibrationStatus {
        val calStatus = MagnetometerCalibrationStatus()
        if (drone != null) {
            val magCalImpl = drone.magnetometerCalibration
            calStatus.isCalibrationCancelled = magCalImpl!!.isCancelled()
            val calibrationInfo: Collection<MagnetometerCalibrationImpl.Info> = magCalImpl.magCalibrationTracker.values
            for (info in calibrationInfo) {
                calStatus.addCalibrationProgress(getMagnetometerCalibrationProgress(info.calProgress))
                calStatus.addCalibrationResult(getMagnetometerCalibrationResult(info.calReport))
            }
        }
        return calStatus
    }

    @JvmStatic
    fun getMagnetometerCalibrationProgress(msgProgress: msg_mag_cal_progress?): MagnetometerCalibrationProgress? {
        return if (msgProgress == null) null else MagnetometerCalibrationProgress(msgProgress.compass_id.toInt(), msgProgress.completion_pct.toInt(),
                msgProgress.direction_x, msgProgress.direction_y, msgProgress.direction_z)
    }

    @JvmStatic
    fun getMagnetometerCalibrationResult(msgReport: msg_mag_cal_report?): MagnetometerCalibrationResult? {
        return if (msgReport == null) null else MagnetometerCalibrationResult(msgReport.compass_id.toInt(), msgReport.cal_status.toInt() == MAG_CAL_STATUS.MAG_CAL_SUCCESS, msgReport.autosaved.toInt() == 1, msgReport.fitness,
                msgReport.ofs_x, msgReport.ofs_y, msgReport.ofs_z,
                msgReport.diag_x, msgReport.diag_y, msgReport.diag_z,
                msgReport.offdiag_x, msgReport.offdiag_y, msgReport.offdiag_z)
    }

    @JvmStatic
    fun startVideoStream(drone: Drone?, videoProps: Bundle?, appId: String?, videoTag: String?,
                         videoSurface: Surface?, listener: ICommandListener?) {
        if (drone !is GenericMavLinkDrone) {
            postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
            return
        }
        drone.startVideoStream(videoProps, appId, videoTag, videoSurface, listener)
    }

    @JvmStatic
    fun stopVideoStream(drone: Drone?, appId: String?, videoTag: String?,
                        listener: ICommandListener?) {
        if (drone !is GenericMavLinkDrone) {
            postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
            return
        }
        drone.stopVideoStream(appId, videoTag, listener)
    }

    @JvmStatic
    fun startVideoStreamForObserver(drone: Drone?, appId: String?, videoTag: String?,
                                    listener: ICommandListener?) {
        if (drone !is GenericMavLinkDrone) {
            postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
            return
        }
        drone.startVideoStreamForObserver(appId, videoTag, listener)
    }

    @JvmStatic
    fun stopVideoStreamForObserver(drone: Drone?, appId: String?, videoTag: String?,
                                   listener: ICommandListener?) {
        if (drone !is GenericMavLinkDrone) {
            postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
            return
        }
        drone.stopVideoStreamForObserver(appId, videoTag, listener)
    }
}
