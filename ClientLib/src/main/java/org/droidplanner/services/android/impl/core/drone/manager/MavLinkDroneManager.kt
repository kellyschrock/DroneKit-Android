package org.droidplanner.services.android.impl.core.drone.manager

import android.content.Context
import android.location.Location
import android.net.Uri
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import org.droidplanner.services.android.impl.core.drone.DroneManager
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.calibration.MagnetometerCalibrationImpl.OnMagnetometerCalibrationListener
import org.droidplanner.services.android.impl.core.gcs.follow.Follow
import org.droidplanner.services.android.impl.core.gcs.ReturnToMe
import org.droidplanner.services.android.impl.communication.service.MAVLinkClient
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkMsgHandler
import org.droidplanner.services.android.impl.core.gcs.GCSHeartbeat
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import timber.log.Timber
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.ArduCopter
import org.droidplanner.services.android.impl.utils.AndroidApWarningParser
import android.os.Handler
import com.MAVLink.MAVLinkPacket
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.ardupilotmega.msg_mag_cal_progress
import com.MAVLink.ardupilotmega.msg_mag_cal_report
import com.MAVLink.common.msg_command_ack
import com.o3dr.services.android.lib.drone.connection.ConnectionType
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.sololink.SoloLinkManager
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.ArduSolo
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.ArduPlane
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.ArduRover
import org.droidplanner.services.android.impl.core.drone.autopilot.px4.Px4Native
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone
import org.droidplanner.services.android.impl.core.gcs.location.FusedLocation
import com.google.android.gms.location.LocationRequest
import org.droidplanner.services.android.impl.utils.prefs.DroidPlannerPrefs
import org.droidplanner.services.android.impl.api.DroneApi
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces
import com.o3dr.services.android.lib.drone.action.GimbalActions
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import org.droidplanner.services.android.impl.api.DroneApi.ClientInfo
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import org.droidplanner.services.android.impl.utils.CommonApiUtils
import com.o3dr.services.android.lib.gcs.returnToMe.ReturnToMeState
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.gcs.action.FollowMeActions
import com.o3dr.services.android.lib.gcs.follow.FollowType
import com.o3dr.services.android.lib.gcs.follow.FollowLocationSource
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.drone.action.StateActions
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.core.gcs.follow.FollowAlgorithm.FollowModes
import org.droidplanner.services.android.impl.utils.SoloApiUtils
import java.util.HashMap

private val TAG = MavLinkDroneManager::class.java.simpleName

/** Created by Fredia Huya-Kouadio on 12/17/15. */
class MavLinkDroneManager(
        context: Context,
        connParams: ConnectionParameter,
        handler: Handler)
: DroneManager<MavLinkDrone?, MAVLinkPacket?>(context, connParams, handler), OnMagnetometerCalibrationListener {

    private var followMe: Follow? = null
    private var returnToMe: ReturnToMe? = null
    private val commandTracker: DroneCommandTracker = DroneCommandTracker(handler)
    private val mavClient: MAVLinkClient = MAVLinkClient(context!!, this, connParams, commandTracker)
    private val mavLinkMsgHandler: MavLinkMsgHandler = MavLinkMsgHandler(this)
    private val gcsHeartbeat: GCSHeartbeat = GCSHeartbeat(mavClient, 1)

    fun onVehicleTypeReceived(type: FirmwareType) {
        if (drone != null) {
            return
        }

        val droneId = connectionParameter.uniqueId + ":" + type.type
        when (type) {
            FirmwareType.ARDU_COPTER -> {
                if (isCompanionComputerEnabled) {
                    onVehicleTypeReceived(FirmwareType.ARDU_SOLO)
                    return
                }
                Timber.i("Instantiating ArduCopter autopilot.")
                drone = ArduCopter(droneId, context, mavClient, handler, AndroidApWarningParser(), this)
            }
            FirmwareType.ARDU_SOLO -> {
                Timber.i("Instantiating ArduSolo autopilot.")
                val params = connectionParameter.paramsBundle
                val ip = params!!.getString(ConnectionType.EXTRA_UDP_SERVER_IP, SoloLinkManager.SOLO_LINK_IP)
                Timber.d("solo IP=%s", ip)
                drone = ArduSolo(droneId, ip, context, mavClient, handler, AndroidApWarningParser(), this)
            }
            FirmwareType.ARDU_PLANE -> {
                Timber.i("Instantiating ArduPlane autopilot.")
                drone = ArduPlane(droneId, context, mavClient, handler, AndroidApWarningParser(), this)
            }
            FirmwareType.ARDU_ROVER -> {
                Timber.i("Instantiating ArduPlane autopilot.")
                drone = ArduRover(droneId, context, mavClient, handler, AndroidApWarningParser(), this)
            }
            FirmwareType.PX4_NATIVE -> {
                Timber.i("Instantiating PX4 Native autopilot.")
                drone = Px4Native(droneId, context, handler, mavClient, AndroidApWarningParser(), this)
            }
            FirmwareType.GENERIC -> {
                Timber.i("Instantiating Generic mavlink autopilot.")
                drone = GenericMavLinkDrone(droneId, context, handler, mavClient, AndroidApWarningParser(), this)
            }
        }

        followMe = Follow(this, handler, FusedLocation(context, handler))
        returnToMe = ReturnToMe(this, FusedLocation(context, handler,
                LocationRequest.PRIORITY_HIGH_ACCURACY, 1000L, 1000L, ReturnToMe.UPDATE_MINIMAL_DISPLACEMENT.toFloat()), this)

        val streamRates = drone?.streamRates
        if (streamRates != null) {
            val dpPrefs = DroidPlannerPrefs(context)
            streamRates.setRates(dpPrefs.rates)
        }

        drone?.addDroneListener(this)
        drone?.setAttributeListener(this)
        val parameterManager = drone!!.parameterManager
        parameterManager?.setParameterListener(this)
        val magnetometer = drone?.magnetometerCalibration
        magnetometer?.setListener(this)
    }

    override fun destroy() {
        super.destroy()
        if (followMe != null && followMe?.isEnabled == true) followMe!!.disableFollowMe()
        returnToMe?.disable()
    }

    override fun doConnect(appId: String?, listener: DroneApi?, tlogLoggingUri: Uri?) {
        Timber.d("doConnect(%s, %s)", appId, tlogLoggingUri)
        if (mavClient.isDisconnected) {
            Timber.i("Opening connection for %s", appId)
            mavClient.openConnection()
        } else {
            if (isConnected) {
                drone?.let {
                    listener?.onDroneEvent(DroneInterfaces.DroneEventsType.CONNECTED, it)
                    if (!it.isConnectionAlive) listener?.onDroneEvent(DroneInterfaces.DroneEventsType.HEARTBEAT_TIMEOUT, it)
                }
            }
        }
        mavClient.registerForTLogLogging(appId, tlogLoggingUri)
    }

    override fun doDisconnect(appId: String?, listener: DroneApi?) {
        if (drone is GenericMavLinkDrone) {
            (drone as GenericMavLinkDrone).tryStoppingVideoStream(appId)
        }

        if (listener != null) {
            mavClient.unregisterForTLogLogging(appId)
            if (isConnected) {
                listener.onDroneEvent(DroneInterfaces.DroneEventsType.DISCONNECTED, drone!!)
            }
        }

        if (mavClient.isConnected && connectedApps.isEmpty()) {
            //Reset the gimbal mount mode
            executeAsyncAction(Action(GimbalActions.ACTION_RESET_GIMBAL_MOUNT_MODE), null)
            mavClient.closeConnection()
        }
    }

    private fun handleCommandAck(ack: msg_command_ack?) {
        if (ack != null) {
            commandTracker.onCommandAck(msg_command_ack.MAVLINK_MSG_ID_COMMAND_ACK, ack)
        }
    }

    override fun notifyReceivedData(packet: MAVLinkPacket?) {
        var receivedMsg: MAVLinkMessage? = null
        receivedMsg = try {
            packet?.unpack()
        } catch (ex: Throwable) {
            Timber.e(ex, "Caught error unpacking packet with msgid %d", packet?.msgid)
            null
        }
        if (receivedMsg == null) return
        if (receivedMsg.msgid == msg_command_ack.MAVLINK_MSG_ID_COMMAND_ACK) {
            val commandAck = receivedMsg as msg_command_ack
            handleCommandAck(commandAck)
        } else {
            mavLinkMsgHandler.receiveData(receivedMsg)
            drone?.onMavLinkMessageReceived(receivedMsg)
        }

        if (!connectedApps.isEmpty()) {
            for (droneEventsListener in connectedApps.values) {
                droneEventsListener.onReceivedMavLinkMessage(receivedMsg)
            }
        }
    }

    override fun onConnectionStatus(connectionStatus: LinkConnectionStatus?) {
        super.onConnectionStatus(connectionStatus)
        when (connectionStatus?.statusCode) {
            LinkConnectionStatus.DISCONNECTED -> gcsHeartbeat.setActive(false)
            LinkConnectionStatus.CONNECTED -> gcsHeartbeat.setActive(true)
        }
    }

    override fun getAttribute(clientInfo: ClientInfo?, attributeType: String?): DroneAttribute? {
        return when (attributeType) {
            AttributeType.FOLLOW_STATE -> CommonApiUtils.getFollowState(followMe)
            AttributeType.RETURN_TO_ME_STATE -> if (returnToMe == null) ReturnToMeState() else returnToMe!!.state
            else -> super.getAttribute(clientInfo, attributeType)
        }
    }

    override fun executeAsyncAction(action: Action, listener: ICommandListener?): Boolean {
        val type = action.type
        val data = action.data
        Timber.d("executeAsyncAction(): action=%s", type)
        return when (type) {
            FollowMeActions.ACTION_ENABLE_FOLLOW_ME -> {
                data!!.classLoader = FollowType::class.java.classLoader
                var locationSource: FollowLocationSource = data.getParcelable(FollowMeActions.EXTRA_LOCATION_SOURCE)

                // Default to internal GPS locations
                if (locationSource == null) {
                    locationSource = FollowLocationSource.INTERNAL
                }
                val followType: FollowType = data.getParcelable(FollowMeActions.EXTRA_FOLLOW_TYPE)
                enableFollowMe(followType, locationSource, listener)
                true
            }

            FollowMeActions.ACTION_UPDATE_FOLLOW_PARAMS -> {
                if (followMe != null) {
                    data!!.classLoader = LatLong::class.java.classLoader
                    val followAlgorithm = followMe!!.followAlgorithm
                    if (followAlgorithm != null) {
                        val paramsMap: MutableMap<String, Any?> = HashMap()
                        val dataKeys = data.keySet()
                        for (key in dataKeys) {
                            paramsMap[key] = data[key]
                        }
                        followAlgorithm.updateAlgorithmParams(paramsMap)
                    }
                }
                true
            }

            FollowMeActions.ACTION_DISABLE_FOLLOW_ME -> {
                CommonApiUtils.disableFollowMe(followMe)
                true
            }

            FollowMeActions.ACTION_NEW_EXTERNAL_LOCATION -> {
                data!!.classLoader = Location::class.java.classLoader
                if (followMe != null && data != null) {
                    val loc = data.getParcelable<Location>(FollowMeActions.EXTRA_LOCATION)
                    if (loc != null) {
                        Timber.i("onNewLocation(%s)", loc)
                        followMe!!.onFollowNewLocation(loc)
                    }
                }
                true
            }

            StateActions.ACTION_ENABLE_RETURN_TO_ME -> {
                val isEnabled = data!!.getBoolean(StateActions.EXTRA_IS_RETURN_TO_ME_ENABLED, false)
                if (returnToMe != null) {
                    if (isEnabled) {
                        returnToMe!!.enable(listener)
                    } else {
                        returnToMe!!.disable()
                    }
                    CommonApiUtils.postSuccessEvent(listener)
                } else {
                    CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                }
                true
            }
            else -> super.executeAsyncAction(action, listener)
        }
    }

    private fun enableFollowMe(followType: FollowType, source: FollowLocationSource, listener: ICommandListener?) {
        Timber.d("enableFollowMe(): followType=%s source=%s", followType, source)
        val selectedMode = CommonApiUtils.followTypeToMode(drone!!, followType)
        if (selectedMode != null) {
            if (followMe == null) {
                Timber.d("enableFollowMe(): followMe is null")
                return
            }

            Timber.d("CURRENT: followMe.enabled=%s followMe.state=%s source=%s", followMe!!.isEnabled, followMe!!.state, source)
            followMe?.let { follow ->
                follow.enableFollowMe(source)
                val currentAlg = follow.followAlgorithm
                if (currentAlg.type != selectedMode) {
                    if (selectedMode == FollowModes.SOLO_SHOT &&
                            !SoloApiUtils.isSoloLinkFeatureAvailable(drone, listener)) {
                        Timber.w("FollowType is SOLO_SHOT, but SoloLink is not available.")
                        return
                    }
                    val algo = selectedMode.getAlgorithmType(this, handler)
                    Timber.d("Setting followAlgorithm to %s", algo)
                    followMe!!.setAlgorithm(algo)
                    CommonApiUtils.postSuccessEvent(listener)
                }
                Timber.i("AFTER: followMe.state=%s type=%s", follow.state, followType)
            }
        }
    }

    override fun onCalibrationCancelled() {
        if (connectedApps.isEmpty()) return
        connectedApps.values.forEach { listener -> listener.onCalibrationCancelled()}
    }

    override fun onCalibrationProgress(progress: msg_mag_cal_progress?) {
        if (connectedApps.isEmpty()) return
        connectedApps.values.forEach { listener -> listener.onCalibrationProgress(progress) }
    }

    override fun onCalibrationCompleted(report: msg_mag_cal_report?) {
        connectedApps.values.forEach { it.onCalibrationCompleted(report) }
    }
}
