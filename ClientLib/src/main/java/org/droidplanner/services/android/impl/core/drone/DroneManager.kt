package org.droidplanner.services.android.impl.core.drone

import android.content.Context
import android.net.Uri
import android.os.Bundle
import android.os.Handler
import android.text.TextUtils
import android.util.Log
import com.o3dr.services.android.lib.drone.action.ControlActions
import com.o3dr.services.android.lib.drone.action.GimbalActions
import com.o3dr.services.android.lib.drone.attribute.error.CommandExecutionError
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.drone.connection.ConnectionType
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import com.o3dr.services.android.lib.drone.property.Parameter
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.api.DroneApi
import org.droidplanner.services.android.impl.api.DroneApi.ClientInfo
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkListener
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.*
import org.droidplanner.services.android.impl.core.drone.DroneManager
import org.droidplanner.services.android.impl.core.drone.autopilot.Drone
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.ArduSolo
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.SoloComp.Companion.isAvailable
import org.droidplanner.services.android.impl.core.drone.manager.MavLinkDroneManager
import org.droidplanner.services.android.impl.utils.CommonApiUtils
import java.util.concurrent.ConcurrentHashMap

/**
 * Bridge between the communication channel, the drone instance(s), and the connected client(s).
 */
open class DroneManager<T : Drone?, D> protected constructor(
        protected val context: Context,
        val connectionParameter: ConnectionParameter,
        protected val handler: Handler)
:   DataLinkListener<D>,
    OnDroneListener<Drone?>,
    OnParameterManagerListener,
    LogMessageListener,
    AttributeEventListener {

    protected val connectedApps = ConcurrentHashMap<String, DroneApi>()
    var drone: T? = null

    private fun destroyAutopilot() {
        drone?.destroy()
        drone = null
    }

    open fun destroy() {
        Log.d(TAG, "Destroying drone manager.")
        disconnect()
        destroyAutopilot()
        connectedApps.clear()
    }

    protected open fun doConnect(appId: String?, listener: DroneApi?, tlogLoggingUri: Uri?) {}
    @Synchronized
    fun connect(appId: String, listener: DroneApi?, tlogLoggingUri: Uri?) {
        if (listener == null || TextUtils.isEmpty(appId)) {
            return
        }
        connectedApps[appId] = listener
        doConnect(appId, listener, tlogLoggingUri)
    }

    private fun disconnect() {
        if (!connectedApps.isEmpty()) {
            for (client in connectedApps.values) {
                disconnect(client.clientInfo)
            }
        }
    }

    /**
     * @return True if we can expect to find a companion computer on the connected channel.
     */
    protected val isCompanionComputerEnabled: Boolean
        protected get() {
            val connectionType = connectionParameter.connectionType
            return (drone is ArduSolo
                    || connectionType == ConnectionType.TYPE_UDP && isAvailable(context)
                    || connectionType == ConnectionType.TYPE_SOLO)
        }

    val connectedAppsCount: Int
        get() = connectedApps.size

    fun disconnect(clientInfo: ClientInfo) {
        val appId = clientInfo.appId
        if (TextUtils.isEmpty(appId)) {
            return
        }
        Log.d(TAG, "Disconnecting client $appId")
        val listener = connectedApps.remove(appId)
        doDisconnect(appId, listener)
    }

    protected open fun doDisconnect(appId: String?, listener: DroneApi?) {
        if (isConnected && listener != null) {
            listener.onDroneEvent(DroneEventsType.DISCONNECTED, drone as Drone)
        }

        if (connectedApps.isEmpty()) {
            //Reset the gimbal mount mode
            executeAsyncAction(null, Action(GimbalActions.ACTION_RESET_GIMBAL_MOUNT_MODE), null)
        }
    }

    protected fun notifyDroneEvent(event: DroneEventsType?) {
        drone?.notifyDroneEvent(event)
    }

    override fun notifyReceivedData(data: D) {}
    override fun onConnectionStatus(connectionStatus: LinkConnectionStatus?) {
        when (connectionStatus?.statusCode) {
            LinkConnectionStatus.DISCONNECTED -> notifyDroneEvent(DroneEventsType.DISCONNECTED)
            LinkConnectionStatus.CONNECTING -> notifyDroneEvent(DroneEventsType.CONNECTING)
        }

        if (connectedApps.isEmpty()) {
            return
        }

        connectionStatus?.let { status ->
            connectedApps.values.forEach { listener -> listener.onConnectionStatus(status) }
        }
    }

//    fun getDrone(): T {
//        return drone
//    }

    val isConnected: Boolean
        get() = (true == drone?.isConnected)

    open fun getAttribute(clientInfo: ClientInfo?, attributeType: String?): DroneAttribute? {
        return drone?.getAttribute(attributeType)
    }

    protected open fun executeAsyncAction(action: Action, listener: ICommandListener?): Boolean {
        return when (action.type) {
            ControlActions.ACTION_ENABLE_MANUAL_CONTROL -> {
                if (drone != null) {
                    drone!!.executeAsyncAction(action, listener)
                } else {
                    CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                }
                true
            }
            else -> if (drone != null) {
                drone!!.executeAsyncAction(action, listener)
            } else {
                CommonApiUtils.postErrorEvent(CommandExecutionError.COMMAND_FAILED, listener)
                true
            }
        }
    }

    fun executeAsyncAction(clientInfo: ClientInfo?, action: Action, listener: ICommandListener?): Boolean {
        when (action.type) {
            ControlActions.ACTION_ENABLE_MANUAL_CONTROL -> action.data?.putString(EXTRA_CLIENT_APP_ID, clientInfo!!.appId)
        }

        return executeAsyncAction(action, listener)
    }

    protected fun notifyDroneAttributeEvent(attributeEvent: String?, eventInfo: Bundle?) {
        if (TextUtils.isEmpty(attributeEvent) || connectedApps.isEmpty()) {
            return
        }
        for (listener in connectedApps.values) {
            listener.onAttributeEvent(attributeEvent!!, eventInfo!!)
        }
    }

    override fun onDroneEvent(event: DroneEventsType, drone: Drone?) {
        var event: DroneEventsType = event

        when (event) {
            DroneEventsType.HEARTBEAT_FIRST,
            DroneEventsType.CONNECTED -> event = DroneEventsType.CONNECTED
        }

        drone?.let {
            connectedApps.values.forEach { listener ->
                listener.onDroneEvent(event, it)
            }
        }
    }

    override fun onBeginReceivingParameters() {
        connectedApps.values.forEach { listener -> listener.onBeginReceivingParameters() }
    }

    override fun onParameterReceived(parameter: Parameter, index: Int, count: Int) {
        connectedApps.values.forEach { listener -> listener.onParameterReceived(parameter, index, count) }
    }

    override fun onEndReceivingParameters() {
        connectedApps.values.forEach { listener -> listener.onEndReceivingParameters() }
    }

    override fun onMessageLogged(logLevel: Int, message: String) {
        connectedApps.values.forEach { listener -> listener.onMessageLogged(logLevel, message) }
    }

    override fun onAttributeEvent(attributeEvent: String, eventInfo: Bundle) {
        notifyDroneAttributeEvent(attributeEvent, eventInfo)
    }

    companion object {
        private val TAG = DroneManager::class.java.simpleName
        const val EXTRA_CLIENT_APP_ID = "extra_client_app_id"

        @JvmStatic
        fun generateDroneManager(context: Context, connParams: ConnectionParameter, handler: Handler): DroneManager<*, *> {
            return when (connParams.connectionType) {
                else -> MavLinkDroneManager(context, connParams, handler)
            }
        }
    }
}
