package org.droidplanner.services.android.impl.core.drone.profiles

import android.content.Context
import android.os.Handler
import android.text.TextUtils
import android.util.SparseBooleanArray
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.msg_param_value
import com.o3dr.services.android.lib.drone.property.Parameter
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkParameters
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.*
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.utils.file.IO.ParameterMetadataLoader
import timber.log.Timber
import java.util.*
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Class to manage the communication of parameters to the MAV.
 *
 *
 * Should be initialized with a MAVLink Object, so the manager can send messages
 * via the MAV link. The function processMessage must be called with every new
 * MAV Message.
 */
class ParameterManager(
        myDrone: MavLinkDrone,
        private val context: Context,
        private val watchdog: Handler)
: DroneVariable<MavLinkDrone?>(myDrone), OnDroneListener<MavLinkDrone?> {

    private val parametersReceiptStartNotification = Runnable { if (parameterListener != null) parameterListener!!.onBeginReceivingParameters() }
    val watchdogCallback = Runnable { onParameterStreamStopped() }
    private val parametersReceiptEndNotification = Runnable { if (parameterListener != null) parameterListener!!.onEndReceivingParameters() }
    private val isRefreshing = AtomicBoolean(false)
    private var expectedParams = 0
    private val paramsRollCall = SparseBooleanArray()
    private val parameters = ConcurrentHashMap<String, Parameter>()
    private val parametersMetadata = ConcurrentHashMap<String?, ParameterMetadata>()
    private var parameterListener: OnParameterManagerListener? = null
    fun refreshParameters() {
        Timber.d("refreshParameters()")
        if (isRefreshing.compareAndSet(false, true)) {
            expectedParams = 0
            parameters.clear()
            paramsRollCall.clear()
            notifyParametersReceiptStart()
            MavLinkParameters.requestParametersList(myDrone)
            resetWatchdog()
        }
    }

    fun getParameters(): Map<String, Parameter> {
        //Update the cache if it's stale. Parameters download is expensive, but we assume the caller knows what it's
        // doing.
        if (parameters.isEmpty()) refreshParameters()
        return parameters
    }

    /**
     * Try to process a Mavlink message if it is a parameter related message
     *
     * @param msg Mavlink message to process
     * @return Returns true if the message has been processed
     */
    fun processMessage(msg: MAVLinkMessage): Boolean {
        if (msg.msgid == msg_param_value.MAVLINK_MSG_ID_PARAM_VALUE) {
            processReceivedParam(msg as msg_param_value)
            return true
        }
        return false
    }

    protected fun processReceivedParam(m_value: msg_param_value) {
        // collect params in parameter list
        val param = Parameter(m_value.param_Id, m_value.param_value.toDouble(), m_value.param_type.toInt())
        loadParameterMetadata(param)
        parameters[param.name!!.toLowerCase(Locale.US)] = param
        val paramIndex = m_value.param_index
        if ( /*(paramIndex == -1) || */paramIndex == m_value.param_count - 1) {
            // update listener
            notifyParameterReceipt(param, 0, 1)
            notifyParametersReceiptEnd()
            return
        }
        paramsRollCall.append(paramIndex, true)
        expectedParams = m_value.param_count

        // update listener
        notifyParameterReceipt(param, paramIndex, m_value.param_count)

        // Are all parameters here? Notify the listener with the parameters
        if (parameters.size >= m_value.param_count) {
            killWatchdog()
            isRefreshing.set(false)
            notifyParametersReceiptEnd()
        } else {
            resetWatchdog()
        }
    }

    private fun reRequestMissingParams(howManyParams: Int) {
        for (i in 0 until howManyParams) {
            if (!paramsRollCall[i]) {
                MavLinkParameters.readParameter(myDrone, i)
            }
        }
    }

    fun sendParameter(parameter: Parameter?) {
        MavLinkParameters.sendParameter(myDrone, parameter)
    }

    fun readParameter(name: String?) {
        MavLinkParameters.readParameter(myDrone, name)
    }

    fun getParameter(name: String): Parameter? {
        return if (TextUtils.isEmpty(name)) null else parameters[name.toLowerCase(Locale.US)]
    }

    private fun onParameterStreamStopped() {
        if (expectedParams > 0) {
            reRequestMissingParams(expectedParams)
            resetWatchdog()
        } else {
            isRefreshing.set(false)
        }
    }

    private fun resetWatchdog() {
        watchdog.removeCallbacks(watchdogCallback)
        watchdog.postDelayed(watchdogCallback, TIMEOUT)
    }

    private fun killWatchdog() {
        watchdog.removeCallbacks(watchdogCallback)
        isRefreshing.set(false)
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.HEARTBEAT_FIRST -> {}
            DroneEventsType.DISCONNECTED, DroneEventsType.HEARTBEAT_TIMEOUT -> killWatchdog()
            DroneEventsType.TYPE -> refreshParametersMetadata()
            else -> {}
        }
    }

    private fun refreshParametersMetadata() {
        //Reload the vehicle parameters metadata
        val metadataType = myDrone!!.firmwareType!!.parameterMetadataGroup
        if (!TextUtils.isEmpty(metadataType)) {
            try {
                ParameterMetadataLoader.load(context, metadataType, parametersMetadata)
            } catch (e: Exception) {
                Timber.e(e, e.message)
            }
        }
        if (parametersMetadata.isEmpty() || parameters.isEmpty()) return
        for (parameter in parameters.values) {
            loadParameterMetadata(parameter)
        }
    }

    private fun loadParameterMetadata(parameter: Parameter) {
        val metadata = parametersMetadata[parameter.name]
        if (metadata != null) {
            parameter.displayName = metadata.displayName
            parameter.description = metadata.description
            parameter.units = metadata.units
            parameter.range = metadata.range
            parameter.values = metadata.values
        }
    }

    fun setParameterListener(parameterListener: OnParameterManagerListener?) {
        this.parameterListener = parameterListener
    }

    private fun notifyParametersReceiptStart() {
        if (parameterListener != null) watchdog.post(parametersReceiptStartNotification)
    }

    private fun notifyParametersReceiptEnd() {
        if (parameterListener != null) watchdog.post(parametersReceiptEndNotification)
    }

    private fun notifyParameterReceipt(parameter: Parameter, index: Int, count: Int) {
        if (parameterListener != null) {
            watchdog.post { if (parameterListener != null) parameterListener!!.onParameterReceived(parameter, index, count) }
        }
    }

    companion object {
        private const val TIMEOUT = 1000L //milliseconds
    }

    init {
        myDrone.addDroneListener(this)
        refreshParametersMetadata()
    }
}
