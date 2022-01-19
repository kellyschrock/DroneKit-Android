package com.o3dr.android.client

import android.content.Context
import android.os.*
import android.os.IBinder.DeathRecipient
import android.util.Log
import com.o3dr.android.client.apis.MissionApi
import com.o3dr.android.client.apis.VehicleApi
import com.o3dr.android.client.interfaces.DroneListener
import com.o3dr.android.client.interfaces.LinkListener
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.calibration.magnetometer.MagnetometerCalibrationStatus
import com.o3dr.services.android.lib.drone.companion.solo.SoloAttributes
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.drone.mission.Mission
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.MissionItem.ComplexItem
import com.o3dr.services.android.lib.drone.property.*
import com.o3dr.services.android.lib.gcs.follow.FollowState
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.gcs.link.LinkEvent
import com.o3dr.services.android.lib.gcs.link.LinkEventExtra
import com.o3dr.services.android.lib.gcs.returnToMe.ReturnToMeState
import com.o3dr.services.android.lib.model.AbstractCommandListener
import com.o3dr.services.android.lib.model.IDroneApi
import com.o3dr.services.android.lib.model.IObserver
import com.o3dr.services.android.lib.model.action.Action
import timber.log.Timber
import java.util.*
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference

/**
 * Created by fhuya on 11/4/14.
 */
open class Drone(val context: Context) {
    interface OnAttributeRetrievedCallback<T : Parcelable?> {
        fun onRetrievalSucceed(attribute: T)
        fun onRetrievalFailed()
    }

    class AttributeRetrievedListener<T : Parcelable?> : OnAttributeRetrievedCallback<T> {
        override fun onRetrievalSucceed(attribute: T) {}
        override fun onRetrievalFailed() {}
    }

    interface OnMissionItemsBuiltCallback<T : MissionItem?> {
        fun onMissionItemsBuilt(complexItems: Array<ComplexItem<T>?>?)
    }

    private val binderDeathRecipient = DeathRecipient { notifyDroneServiceInterrupted("Lost access to the drone api.") }
    private val droneListeners = ConcurrentLinkedQueue<DroneListener>()
    var handler: Handler? = null
        private set
    private var serviceMgr: ControlTower? = null
    private var droneObserver: DroneObserver? = null
    private val droneApiRef = AtomicReference<IDroneApi?>(null)
    var connectionParameter: ConnectionParameter? = null
        private set
    private var linkListener: LinkListener? = null
    private var asyncScheduler: ExecutorService? = null

    // flightTimer
    // ----------------
    private var startTime: Long = 0
    private var elapsedFlightTime: Long = 0
    private val contextClassLoader: ClassLoader

    fun init(controlTower: ControlTower?, handler: Handler?) {
        this.handler = handler
        serviceMgr = controlTower
        droneObserver = DroneObserver(this)
    }

    @Synchronized
    fun start() {
        Timber.d("start()")
        check(serviceMgr!!.isTowerConnected) { "Service manager must be connected." }
        var droneApi = droneApiRef.get()
        if (isStarted(droneApi)) {
            return
        }
        try {
            droneApi = serviceMgr!!.registerDroneApi()
            droneApi.asBinder().linkToDeath(binderDeathRecipient, 0)
        } catch (e: RemoteException) {
            Timber.e(e, e.message)
            throw IllegalStateException("Unable to retrieve a valid drone handle.")
        }
        if (asyncScheduler == null || asyncScheduler!!.isShutdown) {
            asyncScheduler = Executors.newFixedThreadPool(1)
        }
        addAttributesObserver(droneApi, droneObserver)
        resetFlightTimer()
        Timber.d("set droneApiRef to %s", droneApi)
        droneApiRef.set(droneApi)
    }

    @Synchronized
    fun destroy() {
        val droneApi = droneApiRef.get()
        removeAttributesObserver(droneApi, droneObserver)
        try {
            if (isStarted(droneApi)) {
                droneApi!!.asBinder().unlinkToDeath(binderDeathRecipient, 0)
                serviceMgr!!.releaseDroneApi(droneApi)
            }
        } catch (e: RemoteException) {
            Log.e(TAG, e.message, e)
        } catch (e: NoSuchElementException) {
            Log.e(TAG, e.message, e)
        }
        if (asyncScheduler != null) {
            asyncScheduler!!.shutdownNow()
            asyncScheduler = null
        }
        droneApiRef.set(null)
    }

    private fun checkForGroundCollision() {
        val speed = getAttribute<Speed>(AttributeType.SPEED)
        val altitude = getAttribute<Altitude>(AttributeType.ALTITUDE)
        if (speed == null || altitude == null) {
            return
        }
        val verticalSpeed = speed.verticalSpeed
        val altitudeValue = altitude.altitude
        val isCollisionImminent = (altitudeValue
                + verticalSpeed * COLLISION_SECONDS_BEFORE_COLLISION < 0
                ) && verticalSpeed < COLLISION_DANGEROUS_SPEED_METERS_PER_SECOND && altitudeValue > COLLISION_SAFE_ALTITUDE_METERS
        val extrasBundle = Bundle(1)
        extrasBundle.putBoolean(EXTRA_IS_GROUND_COLLISION_IMMINENT, isCollisionImminent)
        notifyAttributeUpdated(ACTION_GROUND_COLLISION_IMMINENT, extrasBundle)
    }

    private fun handleRemoteException(e: RemoteException) {
        val droneApi = droneApiRef.get()
        if (droneApi != null && !droneApi.asBinder().pingBinder()) {
            val errorMsg = e.message
            Log.e(TAG, errorMsg, e)
            notifyDroneServiceInterrupted(errorMsg)
        }
    }

    val speedParameter: Double
        get() {
            val params = getAttribute<Parameters>(AttributeType.PARAMETERS)
            if (params != null) {
                val speedParam = params.getParameter("WPNAV_SPEED")
                if (speedParam != null) {
                    return speedParam.value
                }
            }
            return 0.0
        }

    /**
     * Causes the Runnable to be added to the message queue.
     *
     * @param action Runnabl that will be executed.
     */
    fun post(action: Runnable?) {
        if (handler == null || action == null) {
            return
        }
        handler!!.post(action)
    }

    /**
     * Reset the vehicle flight timer.
     */
    fun resetFlightTimer() {
        elapsedFlightTime = 0
        startTime = SystemClock.elapsedRealtime()
    }

    private fun stopTimer() {
        // lets calc the final elapsed timer
        elapsedFlightTime += SystemClock.elapsedRealtime() - startTime
        startTime = SystemClock.elapsedRealtime()
    }// calc delta time since last checked

    /**
     * @return Vehicle flight time in seconds.
     */
    val flightTime: Long
        get() {
            val droneState = getAttribute<State>(AttributeType.STATE)
            if (droneState != null && droneState.isFlying) {
                // calc delta time since last checked
                elapsedFlightTime += SystemClock.elapsedRealtime() - startTime
                startTime = SystemClock.elapsedRealtime()
            }
            return elapsedFlightTime / 1000
        }

    open fun <T : Parcelable?> getAttribute(type: String?): T? {
        val droneApi = droneApiRef.get()
        if (!isStarted(droneApi) || type == null) {
            return getAttributeDefaultValue(type)
        }
        var attribute: T? = null
        var carrier: Bundle? = null
        try {
            carrier = droneApi!!.getAttribute(type)
        } catch (e: RemoteException) {
            handleRemoteException(e)
        }
        if (carrier != null) {
            try {
                carrier.classLoader = contextClassLoader
                attribute = carrier.getParcelable(type)
            } catch (e: Exception) {
                Log.e(TAG, e.message, e)
            }
        }
        return attribute ?: getAttributeDefaultValue(type)
    }

    fun <T : Parcelable?> getAttributeAsync(attributeType: String?,
                                            callback: OnAttributeRetrievedCallback<T>?) {
        requireNotNull(callback) { "Callback must be non-null." }
        val droneApi = droneApiRef.get()
        if (!isStarted(droneApi)) {
            handler!!.post { callback.onRetrievalFailed() }
            return
        }
        asyncScheduler!!.execute {
            val attribute: T? = getAttribute(attributeType)
            handler?.post {
                if (attribute == null) {
                    callback.onRetrievalFailed()
                } else {
                    callback.onRetrievalSucceed(attribute)
                }
            }
        }
    }

    private fun <T : Parcelable?> getAttributeDefaultValue(attributeType: String?): T? {
        return if (attributeType == null) {
            null
        } else when (attributeType) {
            AttributeType.ALTITUDE -> Altitude() as T
            AttributeType.GPS -> Gps() as T
            AttributeType.STATE -> State() as T
            AttributeType.PARAMETERS -> Parameters() as T
            AttributeType.SPEED -> Speed() as T
            AttributeType.ATTITUDE -> Attitude() as T
            AttributeType.HOME -> Home() as T
            AttributeType.BATTERY -> Battery() as T
            AttributeType.MISSION -> Mission() as T
            AttributeType.SIGNAL -> Signal() as T
            AttributeType.GUIDED_STATE -> GuidedState() as T
            AttributeType.TYPE -> Type() as T
            AttributeType.FOLLOW_STATE -> FollowState() as T
            AttributeType.MAGNETOMETER_CALIBRATION_STATUS -> MagnetometerCalibrationStatus() as T
            AttributeType.RETURN_TO_ME_STATE -> ReturnToMeState() as T
            AttributeType.AUTOPILOT_VERSION -> AutopilotVersion() as T
            AttributeType.CAMERA, SoloAttributes.SOLO_STATE, SoloAttributes.SOLO_GOPRO_STATE, SoloAttributes.SOLO_GOPRO_STATE_V2 -> null
            else -> null
        }
    }

    /**
     * Connect to a vehicle using a specified [ConnectionParameter] and a [LinkListener]
     * callback.
     *
     * @param connParams Specified parameters to determine how to connect the vehicle.
     * @param linkListener A callback that will update the caller on the state of the link connection.
     */
    @JvmOverloads
    fun connect(connParams: ConnectionParameter?, linkListener: LinkListener? = null) {
        VehicleApi.getApi(this).connect(connParams)
        connectionParameter = connParams
        this.linkListener = linkListener
    }

    /**
     * Disconnect from the vehicle.
     */
    fun disconnect() {
        VehicleApi.getApi(this).disconnect()
        connectionParameter = null
        linkListener = null
    }

    fun performAction(action: Action?): Boolean {
        return performActionOnDroneThread(action, null)
    }

    private fun performActionOnDroneThread(action: Action?, listener: AbstractCommandListener?): Boolean {
        return performActionOnHandler(action, handler, listener)
    }

    private fun performActionOnHandler(action: Action?, handler: Handler?, listener: AbstractCommandListener?): Boolean {
        val droneApi = droneApiRef.get()
        if (isStarted(droneApi)) {
            try {
                droneApi?.executeAction(action, wrapListener(handler, listener))
                return true
            } catch (e: RemoteException) {
                handleRemoteException(e)
            }
        }
        return false
    }

    fun performAsyncAction(action: Action?): Boolean {
        Timber.d("performAsyncAction(%s)", action)
        return performAsyncActionOnDroneThread(action, null)
    }

    open fun performAsyncActionOnDroneThread(action: Action?, listener: AbstractCommandListener?): Boolean {
        Timber.d("performAsyncActionOnDroneThread(%s)", action)
        return performAsyncActionOnHandler(action, handler, listener)
    }

    fun performAsyncActionOnHandler(action: Action?, handler: Handler?, listener: AbstractCommandListener?): Boolean {
        Timber.d("performAsyncActionOnHandler(%s)", action)
        val droneApi = droneApiRef.get()
        Timber.d("droneApi=%s", droneApi)
        if (isStarted(droneApi)) {
            Timber.d("droneApi is started")
            try {
                droneApi!!.executeAsyncAction(action, wrapListener(handler, listener))
                return true
            } catch (e: RemoteException) {
                Timber.e(e, e.message)
                handleRemoteException(e)
            }
        } else {
            Timber.d("droneApi is NOT started")
        }
        return false
    }

    private fun isStarted(droneApi: IDroneApi?): Boolean {
        return droneApi != null && droneApi.asBinder().pingBinder()
    }

    val isStarted: Boolean
        get() = isStarted(droneApiRef.get())

    val isConnected: Boolean
        get() {
            val droneApi = droneApiRef.get()
            val droneState = getAttribute<State>(AttributeType.STATE)
            return isStarted(droneApi) && (droneState?.isConnected == true)
        }

    fun <T : MissionItem?> buildMissionItemsAsync(missionItems: Array<ComplexItem<T>?>?,
                                                  callback: OnMissionItemsBuiltCallback<T>?) {
        requireNotNull(callback) { "Callback must be non-null." }
        if (missionItems == null || missionItems.size == 0) {
            return
        }

        asyncScheduler!!.execute {
            for (missionItem in missionItems) {
                MissionApi.getApi(this@Drone).buildMissionItem(missionItem!!)
            }

            handler?.post { callback.onMissionItemsBuilt(missionItems) }
        }
    }

    fun registerDroneListener(listener: DroneListener?) {
        if (listener == null) {
            return
        }
        if (!droneListeners.contains(listener)) {
            droneListeners.add(listener)
        }
    }

    private fun addAttributesObserver(droneApi: IDroneApi, observer: IObserver?) {
        if (isStarted(droneApi)) {
            try {
                droneApi.addAttributesObserver(observer)
            } catch (e: RemoteException) {
                handleRemoteException(e)
            }
        }
    }

    fun addMavlinkObserver(observer: MavlinkObserver?) {
        val droneApi = droneApiRef.get()
        if (isStarted(droneApi)) {
            try {
                droneApi!!.addMavlinkObserver(observer)
            } catch (e: RemoteException) {
                handleRemoteException(e)
            }
        }
    }

    fun removeMavlinkObserver(observer: MavlinkObserver?) {
        val droneApi = droneApiRef.get()
        if (isStarted(droneApi)) {
            try {
                droneApi!!.removeMavlinkObserver(observer)
            } catch (e: RemoteException) {
                handleRemoteException(e)
            }
        }
    }

    fun unregisterDroneListener(listener: DroneListener?) {
        if (listener == null) {
            return
        }
        droneListeners.remove(listener)
    }

    private fun removeAttributesObserver(droneApi: IDroneApi?, observer: IObserver?) {
        if (isStarted(droneApi)) {
            try {
                droneApi!!.removeAttributesObserver(observer)
            } catch (e: RemoteException) {
                handleRemoteException(e)
            }
        }
    }

    fun notifyAttributeUpdated(attributeEvent: String, extras: Bundle?) {
        //Update the bundle classloader
        if (extras != null) {
            extras.classLoader = contextClassLoader
        }
        when (attributeEvent) {
            AttributeEvent.STATE_UPDATED -> getAttributeAsync(AttributeType.STATE, object : OnAttributeRetrievedCallback<State> {
                override fun onRetrievalSucceed(state: State) {
                    if (state.isFlying) {
                        resetFlightTimer()
                    } else {
                        stopTimer()
                    }
                }

                override fun onRetrievalFailed() {
                    stopTimer()
                }
            })
            AttributeEvent.SPEED_UPDATED -> checkForGroundCollision()
            LinkEvent.LINK_STATE_UPDATED -> {
                sendLinkEventToListener(extras)
                return
            }
        }
        sendDroneEventToListeners(attributeEvent, extras)
    }

    private fun sendDroneEventToListeners(attributeEvent: String, extras: Bundle?) {
        if (droneListeners.isEmpty()) {
            return
        }
        handler!!.post {
            for (listener in droneListeners) {
                try {
                    listener.onDroneEvent(attributeEvent, extras)
                } catch (e: Exception) {
                    Log.e(TAG, e.message, e)
                }
            }
        }
    }

    private fun sendLinkEventToListener(extras: Bundle?) {
        if (linkListener == null) {
            return
        }
        if (extras != null) {
            val status: LinkConnectionStatus = extras.getParcelable(LinkEventExtra.EXTRA_CONNECTION_STATUS)
            if (status != null) {
                handler!!.post {
                    try {
                        linkListener!!.onLinkStateUpdated(status)
                    } catch (ex: Throwable) {
                        Timber.e(ex, ex.message)
                    }
                }
            }
        }
    }

    private fun notifyDroneServiceInterrupted(errorMsg: String?) {
        if (droneListeners.isEmpty()) {
            return
        }

        handler?.post { for (listener in droneListeners) listener.onDroneServiceInterrupted(errorMsg) }
    }

    companion object {
        private val CLAZZ_NAME = Drone::class.java.name
        private val TAG = Drone::class.java.simpleName
        const val COLLISION_SECONDS_BEFORE_COLLISION = 2
        const val COLLISION_DANGEROUS_SPEED_METERS_PER_SECOND = -3.0
        const val COLLISION_SAFE_ALTITUDE_METERS = 1.0
        val ACTION_GROUND_COLLISION_IMMINENT = CLAZZ_NAME + ".ACTION_GROUND_COLLISION_IMMINENT"
        const val EXTRA_IS_GROUND_COLLISION_IMMINENT = "extra_is_ground_collision_imminent"
        private fun wrapListener(handler: Handler?, listener: AbstractCommandListener?): AbstractCommandListener? {
            var wrapperListener = listener
            if (handler != null && listener != null) {
                wrapperListener = object : AbstractCommandListener() {
                    override fun onSuccess() {
                        handler.post(Runnable { listener.onSuccess() })
                    }

                    override fun onError(executionError: Int) {
                        handler.post(Runnable { listener.onError(executionError) })
                    }

                    override fun onTimeout() {
                        handler.post(Runnable { listener.onTimeout() })
                    }
                }
            }
            return wrapperListener
        }
    }

    /**
     * Creates a Drone instance.
     *
     * @param context Application context
     */
    init {
        contextClassLoader = context.classLoader
    }
}
