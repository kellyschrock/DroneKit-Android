package org.droidplanner.services.android.impl.api

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.IBinder.DeathRecipient
import android.os.RemoteException
import android.text.TextUtils
import android.util.Pair
import android.view.Surface
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.ardupilotmega.msg_mag_cal_progress
import com.MAVLink.ardupilotmega.msg_mag_cal_report
import com.MAVLink.common.msg_heartbeat
import com.MAVLink.enums.MAV_COMPONENT
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.action.CameraActions
import com.o3dr.services.android.lib.drone.action.ConnectionActions
import com.o3dr.services.android.lib.drone.action.ExperimentalActions
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.drone.mission.Mission
import com.o3dr.services.android.lib.drone.mission.action.MissionActions
import com.o3dr.services.android.lib.drone.mission.item.command.ResetROI
import com.o3dr.services.android.lib.drone.mission.item.spatial.RegionOfInterest
import com.o3dr.services.android.lib.drone.property.Parameter
import com.o3dr.services.android.lib.drone.property.State
import com.o3dr.services.android.lib.gcs.event.GCSEvent
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus.Companion.newFailedConnectionStatus
import com.o3dr.services.android.lib.gcs.link.LinkEvent
import com.o3dr.services.android.lib.gcs.link.LinkEventExtra
import com.o3dr.services.android.lib.mavlink.MavlinkMessageWrapper
import com.o3dr.services.android.lib.model.*
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.api.DroidPlannerService
import org.droidplanner.services.android.impl.communication.connection.SoloConnection.Companion.getSoloConnectionParameterFromUdp
import org.droidplanner.services.android.impl.communication.connection.SoloConnection.Companion.isUdpSoloConnection
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.*
import org.droidplanner.services.android.impl.core.drone.DroneManager
import org.droidplanner.services.android.impl.core.drone.autopilot.Drone
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.drone.variables.calibration.MagnetometerCalibrationImpl.OnMagnetometerCalibrationListener
import org.droidplanner.services.android.impl.exception.ConnectionException
import org.droidplanner.services.android.impl.utils.CommonApiUtils
import org.droidplanner.services.android.impl.utils.video.VideoManager
import timber.log.Timber
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * Implementation for the IDroneApi interface.
 */
class DroneApi internal constructor(private val service: DroidPlannerService, listener: IApiListener, theOwnerId: String)
    : IDroneApi.Stub(),
    OnDroneListener<Drone>,
    AttributeEventListener,
    OnParameterManagerListener,
    OnMagnetometerCalibrationListener,
    DeathRecipient {

    private val context: Context = service.applicationContext
    private val observersList: ConcurrentLinkedQueue<IObserver> = ConcurrentLinkedQueue()
    private val mavlinkObserversList: ConcurrentLinkedQueue<IMavlinkObserver> = ConcurrentLinkedQueue()
    var droneManager: DroneManager<*, *>? = null
        private set
    private val apiListener: IApiListener = listener
    val ownerId: String = theOwnerId
    val clientInfo: ClientInfo
    private var connectionParams: ConnectionParameter? = null

    fun destroy() {
        Timber.d("Destroying drone api instance for %s", ownerId)
        observersList.clear()
        mavlinkObserversList.clear()
        try {
            apiListener.asBinder().unlinkToDeath(this, 0)
        } catch (e: NoSuchElementException) {
            Timber.e(e, e.message)
        }
        service.disconnectDroneManager(droneManager, clientInfo)
    }

    private val drone: Drone?
        private get() = if (droneManager == null) {
            null
        } else droneManager!!.drone

    @Throws(RemoteException::class)
    override fun getAttribute(type: String): Bundle {
        val carrier = Bundle()
        when (type) {
            AttributeType.CAMERA -> carrier.putParcelable(type, CommonApiUtils.getCameraProxy(drone!!, service.cameraDetails!!))
            else -> if (droneManager != null) {
                val attribute = droneManager!!.getAttribute(clientInfo, type)
                if (attribute != null) {

                    //Check if the client supports the ResetROI mission item.
                    // Replace it with a RegionOfInterest with coordinate set to 0 if it doesn't.
                    if (clientInfo.clientVersionCode < RESET_ROI_LIB_VERSION && attribute is Mission) {
                        val missionItems = attribute.missionItems
                        val missionItemsCount = missionItems.size
                        var i = 0
                        while (i < missionItemsCount) {
                            val missionItem = missionItems[i]
                            if (missionItem is ResetROI) {
                                missionItems.removeAt(i)
                                val replacement = RegionOfInterest()
                                replacement.coordinate = LatLongAlt(0.0, 0.0, 0.0)
                                missionItems.add(i, replacement)
                            }
                            i++
                        }
                    }
                    carrier.putParcelable(type, attribute)
                }
            }
        }
        return carrier
    }

    val isConnected: Boolean
        get() = droneManager != null && droneManager!!.isConnected

    @Throws(ConnectionException::class)
    private fun checkConnectionParameter(connParams: ConnectionParameter?): ConnectionParameter {
        if (connParams == null) {
            throw ConnectionException("Invalid connection parameters")
        }
        if (isUdpSoloConnection(context, connParams)) {
            val update = getSoloConnectionParameterFromUdp(context, connParams)
            if (update != null) {
                return update
            }
        }
        return connParams
    }

    fun connect(connParams: ConnectionParameter) {
        var connParams = connParams
        try {
            //Validate the given connection parameter
            connParams = checkConnectionParameter(connParams)

            //Validate the current connection parameter for the drone
            val currentConnParams = if (connectionParams == null) connectionParams else checkConnectionParameter(connectionParams)
            Timber.d("connect(): equals? %s", connParams.equals(currentConnParams))
            if (!connParams.equals(currentConnParams)) {
                if (droneManager != null) {
                    val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.ADDRESS_IN_USE,
                            "Connection already started with different connection parameters")
                    onConnectionStatus(connectionStatus)
                    return
                }
                connectionParams = connParams
                droneManager = service.connectDroneManager(connectionParams, ownerId, this)
            }
        } catch (e: ConnectionException) {
            Timber.e(e, e.message)
            val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.INVALID_CREDENTIALS, e.message)
            onConnectionStatus(connectionStatus)
            disconnect()
        }
    }

    fun disconnect() {
        service.disconnectDroneManager(droneManager, clientInfo)
        connectionParams = null
        droneManager = null
    }

    private fun checkForSelfRelease() {
        //Check if the apiListener is still connected instead.
        if (!apiListener.asBinder().pingBinder()) {
            Timber.w("Client is not longer available.")
            context.startService(Intent(context, DroidPlannerService::class.java)
                    .setAction(DroidPlannerService.ACTION_RELEASE_API_INSTANCE)
                    .putExtra(DroidPlannerService.EXTRA_API_INSTANCE_APP_ID, ownerId))
        }
    }

    @Throws(RemoteException::class)
    override fun addAttributesObserver(observer: IObserver) {
        if (observer != null) {
            Timber.d("Adding attributes observer.")
            observersList.add(observer)
        }
    }

    @Throws(RemoteException::class)
    override fun removeAttributesObserver(observer: IObserver) {
        if (observer != null) {
            Timber.d("Removing attributes observer.")
            observersList.remove(observer)
            checkForSelfRelease()
        }
    }

    @Throws(RemoteException::class)
    override fun addMavlinkObserver(observer: IMavlinkObserver) {
        if (observer != null) {
            mavlinkObserversList.add(observer)
        }
    }

    @Throws(RemoteException::class)
    override fun removeMavlinkObserver(observer: IMavlinkObserver) {
        if (observer != null) {
            mavlinkObserversList.remove(observer)
            checkForSelfRelease()
        }
    }

    @Throws(RemoteException::class)
    override fun executeAction(action: Action, listener: ICommandListener?) {
        Timber.d("executeAction(): action=%s", action)
        if (action == null) {
            return
        }
        val type = action.type ?: return
        val data = action.data
        if (data != null) {
            data.classLoader = context.classLoader
        }
        val drone = drone
        when (type) {
            ConnectionActions.ACTION_CONNECT -> {
                val param: ConnectionParameter? = data?.getParcelable(ConnectionActions.EXTRA_CONNECT_PARAMETER)
                param?.let {
                    connect(param)
                }
            }

            ConnectionActions.ACTION_DISCONNECT -> disconnect()

            CameraActions.ACTION_START_VIDEO_STREAM -> {
                data?.getParcelable<Surface>(CameraActions.EXTRA_VIDEO_DISPLAY)?.let { videoSurface ->
                    val videoTag = data.getString(CameraActions.EXTRA_VIDEO_TAG, "")
                    var videoProps = data.getBundle(CameraActions.EXTRA_VIDEO_PROPERTIES)
                    if (videoProps == null) {
                        //Only case where it's null is when interacting with a deprecated client version.
                        //In this case, we assume that the client is attempting to start a solo stream, since that's
                        //the only api that was exposed.
                        videoProps = Bundle().apply {
                            putInt(CameraActions.EXTRA_VIDEO_PROPS_UDP_PORT, VideoManager.ARTOO_UDP_PORT)
                        }
                    }

                    CommonApiUtils.startVideoStream(drone, videoProps, ownerId, videoTag, videoSurface, listener)
                }
            }

            ExperimentalActions.ACTION_START_VIDEO_STREAM_FOR_OBSERVER -> {
                data?.getString(CameraActions.EXTRA_VIDEO_TAG, "")?.let { videoTag ->
                    CommonApiUtils.startVideoStreamForObserver(drone, ownerId, videoTag, listener)
                }
            }

            CameraActions.ACTION_STOP_VIDEO_STREAM -> {
                data?.getString(CameraActions.EXTRA_VIDEO_TAG, "")?.let { videoTag ->
                    CommonApiUtils.stopVideoStream(drone, ownerId, videoTag, listener)
                }
            }

            ExperimentalActions.ACTION_STOP_VIDEO_STREAM_FOR_OBSERVER -> {
                data?.getString(CameraActions.EXTRA_VIDEO_TAG, "")?.let { videoTag ->
                    CommonApiUtils.stopVideoStreamForObserver(drone, ownerId, videoTag, listener)
                }
            }

            MissionActions.ACTION_BUILD_COMPLEX_MISSION_ITEM -> if (drone is MavLinkDrone || drone == null) {
                CommonApiUtils.buildComplexMissionItem(drone as MavLinkDrone?, data)
            } else {
                CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_UNSUPPORTED, listener)
            }

            else -> if (droneManager != null) {
                droneManager!!.executeAsyncAction(clientInfo, action, listener)
            } else {
                CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
            }
        }
    }

    @Throws(RemoteException::class)
    override fun executeAsyncAction(action: Action, listener: ICommandListener?) {
        executeAction(action, listener)
    }

    @Throws(RemoteException::class)
    override fun performAction(action: Action) {
        executeAction(action, null)
    }

    @Throws(RemoteException::class)
    override fun performAsyncAction(action: Action) {
        performAction(action)
    }

    private fun notifyAttributeUpdate(attributesInfo: List<Pair<String, Bundle>>?) {
        if (observersList.isEmpty() || attributesInfo == null || attributesInfo.isEmpty()) {
            return
        }
        for (info in attributesInfo) {
            notifyAttributeUpdate(info.first, info.second)
        }
    }

    private fun notifyAttributeUpdate(attributeEvent: String?, extrasBundle: Bundle?) {
        if (observersList.isEmpty()) {
            return
        }
        if (attributeEvent != null) {
            for (observer in observersList) {
                try {
                    observer.onAttributeUpdated(attributeEvent, extrasBundle)
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                    try {
                        removeAttributesObserver(observer)
                    } catch (e1: RemoteException) {
                        Timber.e(e, e1.message)
                    }
                }
            }
        }
    }

    fun onReceivedMavLinkMessage(msg: MAVLinkMessage?) {
        if (mavlinkObserversList.isEmpty()) {
            return
        }
        if (msg != null) {
            val msgWrapper = MavlinkMessageWrapper(msg)
            for (observer in mavlinkObserversList) {

                // TODO: Remove after debugging this crap
                if (msg.compid == MAV_COMPONENT.MAV_COMP_ID_CAMERA && msg.msgid != msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT) {
                    Timber.d("Relaying %d from camera", msg.msgid)
                }
                try {
                    observer.onMavlinkMessageReceived(msgWrapper)
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                    try {
                        removeMavlinkObserver(observer)
                    } catch (e1: RemoteException) {
                        Timber.e(e1, e1.message)
                    }
                }
            }
        }
    }

    fun onMessageLogged(logLevel: Int, message: String?) {
        val args = Bundle(2)
        args.putInt(AttributeEventExtra.EXTRA_AUTOPILOT_MESSAGE_LEVEL, logLevel)
        args.putString(AttributeEventExtra.EXTRA_AUTOPILOT_MESSAGE, message)
        notifyAttributeUpdate(AttributeEvent.AUTOPILOT_MESSAGE, args)
    }

    override fun onAttributeEvent(attributeEvent: String, eventInfo: Bundle) {
        if (TextUtils.isEmpty(attributeEvent)) {
            return
        }
        notifyAttributeUpdate(attributeEvent, eventInfo)
    }

    override fun onDroneEvent(event: DroneEventsType, drone: Drone) {
        val extrasBundle = Bundle()
        var droneId: String? = ""
        if (drone != null) {
            droneId = drone.id
        }
        extrasBundle.putString(AttributeEventExtra.EXTRA_VEHICLE_ID, droneId)
        var droneEvent: String? = null
        val attributesInfo: MutableList<Pair<String, Bundle>> = ArrayList()
        when (event) {
            DroneEventsType.DISCONNECTED -> {
                //Broadcast the disconnection with the vehicle.
                context.sendBroadcast(Intent(GCSEvent.ACTION_VEHICLE_DISCONNECTION)
                        .putExtra(GCSEvent.EXTRA_APP_ID, ownerId))
                droneEvent = AttributeEvent.STATE_DISCONNECTED
            }
            DroneEventsType.GUIDEDPOINT -> droneEvent = AttributeEvent.GUIDED_POINT_UPDATED
            DroneEventsType.RADIO -> droneEvent = AttributeEvent.SIGNAL_UPDATED
            DroneEventsType.RC_IN -> {}
            DroneEventsType.RC_OUT -> {}
            DroneEventsType.ARMING_STARTED, DroneEventsType.ARMING -> droneEvent = AttributeEvent.STATE_ARMING
            DroneEventsType.AUTOPILOT_WARNING -> {
                val droneState = drone.getAttribute(AttributeType.STATE) as State
                if (droneState != null) {
                    extrasBundle.putString(AttributeEventExtra.EXTRA_AUTOPILOT_ERROR_ID, droneState.autopilotErrorId)
                }
                droneEvent = AttributeEvent.AUTOPILOT_ERROR
            }
            DroneEventsType.MODE -> droneEvent = AttributeEvent.STATE_VEHICLE_MODE
            DroneEventsType.ATTITUDE, DroneEventsType.ORIENTATION -> droneEvent = AttributeEvent.ATTITUDE_UPDATED
            DroneEventsType.SPEED -> droneEvent = AttributeEvent.SPEED_UPDATED
            DroneEventsType.BATTERY -> droneEvent = AttributeEvent.BATTERY_UPDATED
            DroneEventsType.STATE -> droneEvent = AttributeEvent.STATE_UPDATED
            DroneEventsType.MISSION_UPDATE -> droneEvent = AttributeEvent.MISSION_UPDATED
            DroneEventsType.MISSION_RECEIVED -> droneEvent = AttributeEvent.MISSION_RECEIVED
            DroneEventsType.FIRMWARE, DroneEventsType.TYPE -> droneEvent = AttributeEvent.TYPE_UPDATED
            DroneEventsType.HOME -> droneEvent = AttributeEvent.HOME_UPDATED
            DroneEventsType.CALIBRATION_IMU -> if (drone is MavLinkDrone) {
                val calIMUMessage = drone.calibrationSetup?.message
                extrasBundle.putString(AttributeEventExtra.EXTRA_CALIBRATION_IMU_MESSAGE, calIMUMessage)
                droneEvent = AttributeEvent.CALIBRATION_IMU
            }
            DroneEventsType.CALIBRATION_TIMEOUT -> if (drone is MavLinkDrone) {
                /*
                 * here we will check if we are in calibration mode but if at
                 * the same time 'msg' is empty - then it is actually not doing
                 * calibration what we should do is to reset the calibration
                 * flag and re-trigger the HEARTBEAT_TIMEOUT this however should
                 * not be happening
                 */
                val accelCalibration = drone.calibrationSetup
                val message = accelCalibration?.message
                droneEvent = if (accelCalibration?.isCalibrating == true && TextUtils.isEmpty(message)) {
                    accelCalibration.cancelCalibration()
                    AttributeEvent.HEARTBEAT_TIMEOUT
                } else {
                    extrasBundle.putString(AttributeEventExtra.EXTRA_CALIBRATION_IMU_MESSAGE, message)
                    AttributeEvent.CALIBRATION_IMU_TIMEOUT
                }
            }
            DroneEventsType.HEARTBEAT_TIMEOUT -> droneEvent = AttributeEvent.HEARTBEAT_TIMEOUT
            DroneEventsType.CONNECTING -> droneEvent = AttributeEvent.STATE_CONNECTING
            DroneEventsType.HEARTBEAT_FIRST -> {
                val heartBeatExtras = Bundle()
                heartBeatExtras.putString(AttributeEventExtra.EXTRA_VEHICLE_ID, drone.id)
                if (drone is MavLinkDrone) {
                    heartBeatExtras.putInt(AttributeEventExtra.EXTRA_MAVLINK_VERSION, drone.mavlinkVersion)
                }
                attributesInfo.add(Pair.create(AttributeEvent.HEARTBEAT_FIRST, heartBeatExtras))
                //Broadcast the vehicle connection.
                val sanitizedParameter = connectionParams!!.clone()
                context.sendBroadcast(Intent(GCSEvent.ACTION_VEHICLE_CONNECTION)
                        .putExtra(GCSEvent.EXTRA_APP_ID, ownerId)
                        .putExtra(GCSEvent.EXTRA_VEHICLE_CONNECTION_PARAMETER, sanitizedParameter))
                attributesInfo.add(Pair.create(AttributeEvent.STATE_CONNECTED, extrasBundle))
            }
            DroneEventsType.CONNECTED -> {
                val sanitizedParameter = connectionParams!!.clone()
                context.sendBroadcast(Intent(GCSEvent.ACTION_VEHICLE_CONNECTION)
                        .putExtra(GCSEvent.EXTRA_APP_ID, ownerId)
                        .putExtra(GCSEvent.EXTRA_VEHICLE_CONNECTION_PARAMETER, sanitizedParameter))
                attributesInfo.add(Pair.create(AttributeEvent.STATE_CONNECTED, extrasBundle))
            }
            DroneEventsType.HEARTBEAT_RESTORED -> {
                if (drone is MavLinkDrone) {
                    extrasBundle.putInt(AttributeEventExtra.EXTRA_MAVLINK_VERSION, drone.mavlinkVersion)
                }
                droneEvent = AttributeEvent.HEARTBEAT_RESTORED
            }
            DroneEventsType.MISSION_SENT -> droneEvent = AttributeEvent.MISSION_SENT
            DroneEventsType.INVALID_POLYGON -> {}
            DroneEventsType.MISSION_WP_UPDATE -> if (drone is MavLinkDrone) {
                val currentWaypoint = drone.missionStats?.currentWP ?: 0
                extrasBundle.putInt(AttributeEventExtra.EXTRA_MISSION_CURRENT_WAYPOINT, currentWaypoint)
                droneEvent = AttributeEvent.MISSION_ITEM_UPDATED
            }
            DroneEventsType.MISSION_WP_REACHED -> if (drone is MavLinkDrone) {
                val lastReachedWaypoint = drone.missionStats?.lastReachedWP ?: 0
                extrasBundle.putInt(AttributeEventExtra.EXTRA_MISSION_LAST_REACHED_WAYPOINT, lastReachedWaypoint)
                droneEvent = AttributeEvent.MISSION_ITEM_REACHED
            }
            DroneEventsType.ALTITUDE -> droneEvent = AttributeEvent.ALTITUDE_UPDATED
            DroneEventsType.WARNING_SIGNAL_WEAK -> droneEvent = AttributeEvent.SIGNAL_WEAK
            DroneEventsType.WARNING_NO_GPS -> droneEvent = AttributeEvent.WARNING_NO_GPS
            DroneEventsType.MAGNETOMETER -> {}
            DroneEventsType.FOOTPRINT -> droneEvent = AttributeEvent.CAMERA_FOOTPRINTS_UPDATED
            DroneEventsType.EKF_STATUS_UPDATE -> droneEvent = AttributeEvent.STATE_EKF_REPORT
            DroneEventsType.EKF_POSITION_STATE_UPDATE -> droneEvent = AttributeEvent.STATE_EKF_POSITION
        }
        droneEvent?.let { notifyAttributeUpdate(it, extrasBundle) }
        if (!attributesInfo.isEmpty()) {
            notifyAttributeUpdate(attributesInfo)
        }
    }

    override fun onBeginReceivingParameters() {
        notifyAttributeUpdate(AttributeEvent.PARAMETERS_REFRESH_STARTED, null)
    }

    override fun onParameterReceived(parameter: Parameter, index: Int, count: Int) {
        val paramsBundle = Bundle(4)
        paramsBundle.putInt(AttributeEventExtra.EXTRA_PARAMETER_INDEX, index)
        paramsBundle.putInt(AttributeEventExtra.EXTRA_PARAMETERS_COUNT, count)
        paramsBundle.putString(AttributeEventExtra.EXTRA_PARAMETER_NAME, parameter.name)
        paramsBundle.putDouble(AttributeEventExtra.EXTRA_PARAMETER_VALUE, parameter.value)
        paramsBundle.putInt(AttributeEventExtra.EXTRA_PARAMETER_TYPE, parameter.type)
        notifyAttributeUpdate(AttributeEvent.PARAMETER_RECEIVED, paramsBundle)
    }

    override fun onEndReceivingParameters() {
        notifyAttributeUpdate(AttributeEvent.PARAMETERS_REFRESH_COMPLETED, null)
    }

    fun onConnectionStatus(connectionStatus: LinkConnectionStatus) {
        when (connectionStatus.statusCode) {
            LinkConnectionStatus.FAILED -> {
                disconnect()
                checkForSelfRelease()
            }
            LinkConnectionStatus.DISCONNECTED -> {
                disconnect()
                checkForSelfRelease()
            }
        }
        val extras = Bundle()
        extras.putParcelable(LinkEventExtra.EXTRA_CONNECTION_STATUS, connectionStatus)
        notifyAttributeUpdate(LinkEvent.LINK_STATE_UPDATED, extras)
    }

    override fun binderDied() {
        checkForSelfRelease()
    }

    override fun onCalibrationCancelled() {
        notifyAttributeUpdate(AttributeEvent.CALIBRATION_MAG_CANCELLED, null)
    }

    override fun onCalibrationProgress(progress: msg_mag_cal_progress?) {
        val progressBundle = Bundle(1)
        progressBundle.putParcelable(AttributeEventExtra.EXTRA_CALIBRATION_MAG_PROGRESS,
                CommonApiUtils.getMagnetometerCalibrationProgress(progress))
        notifyAttributeUpdate(AttributeEvent.CALIBRATION_MAG_PROGRESS, progressBundle)
    }

    override fun onCalibrationCompleted(report: msg_mag_cal_report?) {
        val reportBundle = Bundle(1)
        reportBundle.putParcelable(AttributeEventExtra.EXTRA_CALIBRATION_MAG_RESULT,
                CommonApiUtils.getMagnetometerCalibrationResult(report))
        notifyAttributeUpdate(AttributeEvent.CALIBRATION_MAG_COMPLETED, reportBundle)
    }

    class ClientInfo(val appId: String, val apiVersionCode: Int, val clientVersionCode: Int)
    companion object {
        //The Reset ROI mission item was introduced in version 2.6.8. Any client library older than this do not support it.
        private const val RESET_ROI_LIB_VERSION = 206080
    }

    init {
        var apiVersionCode = -1
        var clientVersionCode = -1
        try {
            apiListener.asBinder().linkToDeath(this, 0)
            checkForSelfRelease()
            apiVersionCode = apiListener.apiVersionCode
            clientVersionCode = apiListener.clientVersionCode
        } catch (e: RemoteException) {
            Timber.e(e, e.message)
            service.releaseDroneApi(this.ownerId)
        }
        clientInfo = ClientInfo(this.ownerId, apiVersionCode, clientVersionCode)
    }
}
