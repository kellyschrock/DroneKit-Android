package org.droidplanner.services.android.impl.api

import android.annotation.SuppressLint
import android.annotation.TargetApi
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.support.v4.app.NotificationCompat
import android.support.v4.content.LocalBroadcastManager
import android.text.TextUtils
import android.util.Log
import com.o3dr.android.client.R
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.drone.mission.item.complex.CameraDetail
import com.o3dr.services.android.lib.model.IApiListener
import com.o3dr.services.android.lib.model.IDroidPlannerService
import org.droidplanner.services.android.impl.api.DroneApi.ClientInfo
import org.droidplanner.services.android.impl.core.drone.DroneManager
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import org.droidplanner.services.android.impl.utils.Utils
import org.droidplanner.services.android.impl.utils.file.IO.CameraInfoLoader
import timber.log.Timber
import java.util.*
import java.util.concurrent.ConcurrentHashMap

/** 3DR Services background service implementation. */
class DroidPlannerService : Service() {
    /**
     * Used to broadcast service events.
     */
    private var lbm: LocalBroadcastManager? = null

    /**
     * Stores drone api instances per connected client. The client are denoted by their app id.
     */
    @JvmField
    val droneApiStore = ConcurrentHashMap<String, DroneApi>()

    /**
     * Caches drone managers per connection type.
     */
    val droneManagers = ConcurrentHashMap<ConnectionParameter, DroneManager<*, *>>()
    private var dpServices: DPServices? = null
    private var droneAccess: DroneAccess? = null
    private var cameraInfoLoader: CameraInfoLoader? = null
    private var cachedCameraDetails: List<CameraDetail>? = null

    /**
     * Generate a drone api instance for the client denoted by the given app id.
     *
     * @param listener Used to retrieve api information.
     * @param appId    Application id of the connecting client.
     * @return a IDroneApi instance
     */
    fun registerDroneApi(listener: IApiListener, appId: String): DroneApi {
        Timber.d("registerDroneApi(): listener=%s appId=%s", listener, appId)
        val droneApi = DroneApi(this, listener!!, appId)
        droneApiStore[appId] = droneApi
        lbm!!.sendBroadcast(Intent(ACTION_DRONE_CREATED))
        updateForegroundNotification()
        Timber.d("registerDroneApi(): droneApi=%s", droneApi)
        return droneApi
    }

    /**
     * Release the drone api instance attached to the given app id.
     *
     * @param appId Application id of the disconnecting client.
     */
    fun releaseDroneApi(appId: String?) {
        Timber.d("releaseDroneApi(%s)", appId)
        if (appId == null) return
        val droneApi = droneApiStore.remove(appId)
        if (droneApi != null) {
            Timber.d("Releasing drone api instance for $appId")
            droneApi.destroy()
            lbm!!.sendBroadcast(Intent(ACTION_DRONE_DESTROYED))
            updateForegroundNotification()
        }
    }

    /**
     * Establish a connection with a vehicle using the given connection parameter.
     *
     * @param connParams Parameters used to connect to the vehicle.
     * @param appId      Application id of the connecting client.
     * @param listener   Callback to receive drone events.
     * @return A DroneManager instance which acts as router between the connected vehicle and the listeneing client(s).
     */
    fun connectDroneManager(connParams: ConnectionParameter?, appId: String, listener: DroneApi?): DroneManager<*, *>? {
        Timber.d("connectDroneManager(%s, %s)", connParams, appId)
        if (connParams == null || TextUtils.isEmpty(appId) || listener == null) return null
        var droneMgr = droneManagers[connParams]
        if (droneMgr == null) {
            val temp = DroneManager.generateDroneManager(applicationContext, connParams, Handler(Looper.getMainLooper()))
            droneMgr = droneManagers.putIfAbsent(connParams, temp)
            if (droneMgr == null) {
                Timber.d("Generating new drone manager.")
                droneMgr = temp
            } else {
                temp.destroy()
            }
        }
        Timber.d("Drone manager connection for $appId")
        droneMgr?.connect(appId, listener, connParams.tLogLoggingUri)
        return droneMgr
    }

    /**
     * Disconnect the given client from the vehicle managed by the given drone manager.
     *
     * @param droneMgr   Handler for the connected vehicle.
     * @param clientInfo Info of the disconnecting client.
     */
    fun disconnectDroneManager(droneMgr: DroneManager<*, *>?, clientInfo: ClientInfo?) {
        Timber.d("disconnectDroneManager()")
        if (droneMgr == null || clientInfo == null || TextUtils.isEmpty(clientInfo.appId)) return
        val appId = clientInfo.appId
        Timber.d("Drone manager disconnection for $appId")
        droneMgr.disconnect(clientInfo)
        if (droneMgr.connectedAppsCount == 0) {
            Timber.d("Destroying drone manager.")
            droneMgr.destroy()
            droneManagers.remove(droneMgr.connectionParameter)
        }
    }

    /**
     * Retrieves the set of camera info provided by the app.
     *
     * @return a list of [CameraDetail] objects.
     */
    @get:Synchronized
    val cameraDetails: List<CameraDetail>?
        get() {
            if (cachedCameraDetails == null) {
                val cameraInfoNames = cameraInfoLoader!!.cameraInfoList
                val cameraInfos: MutableList<CameraInfo> = ArrayList(cameraInfoNames.size)
                for (infoName in cameraInfoNames) {
                    try {
                        cameraInfos.add(cameraInfoLoader!!.openFile(infoName))
                    } catch (e: Exception) {
                        Timber.e(e, e.message)
                    }
                }
                val cameraDetails: MutableList<CameraDetail> = ArrayList(cameraInfos.size)
                for (camInfo in cameraInfos) {
                    cameraDetails.add(CameraDetail(camInfo.name, camInfo.sensorWidth,
                            camInfo.sensorHeight, camInfo.sensorResolution, camInfo.focalLength,
                            camInfo.overlap, camInfo.sidelap, camInfo.isInLandscapeOrientation))
                }
                cachedCameraDetails = cameraDetails
            }
            return cachedCameraDetails
        }

    override fun onBind(intent: Intent): IBinder {
        Timber.d("Binding intent: $intent")
        val action = intent.action
        return if (IDroidPlannerService::class.java.name == action) {
            // Return binder to ipc client-server interaction.
            dpServices!!
        } else {
            // Return binder to the service.
            droneAccess!!
        }
    }

    @SuppressLint("NewApi")
    override fun onCreate() {
        super.onCreate()
        Timber.d("Creating %s", TAG)
        val context = applicationContext
        droneAccess = DroneAccess(this)
        dpServices = DPServices(this)
        lbm = LocalBroadcastManager.getInstance(context)
        cameraInfoLoader = CameraInfoLoader(context)
        updateForegroundNotification()
    }

    @SuppressLint("NewApi")
    private fun updateForegroundNotification() {
        Timber.d("updateForegroundNotification()")
        val context = applicationContext
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            Log.v(TAG, "Using new-style notifications")
            val channelId = createNotificationChannel("droidPlannerService", "DroidPlannerService")

            //Put the service in the foreground
            val notifBuilder = Notification.Builder(context)
                    .setContentTitle("Dronekit-Android")
                    .setPriority(Notification.PRIORITY_MIN)
                    .setSmallIcon(R.drawable.ic_stat_notify)
                    .setCategory(Notification.CATEGORY_SERVICE)
                    .setChannelId(channelId)
            val connectedCount = droneApiStore.size
            if (connectedCount > 0) {
                if (connectedCount == 1) {
                    notifBuilder.setContentText("1 connected app")
                } else {
                    notifBuilder.setContentText("$connectedCount connected apps")
                }
            }
            val notification = notifBuilder.build()
            startForeground(FOREGROUND_ID, notification)
        } else {
            Log.v(TAG, "Using old-style notifications")
            //Put the service in the foreground
            val notifBuilder = NotificationCompat.Builder(context)
                    .setContentTitle("Dronekit-Android")
                    .setPriority(NotificationCompat.PRIORITY_MIN)
                    .setSmallIcon(R.drawable.ic_stat_notify)
                    .setCategory(Notification.CATEGORY_SERVICE)
            val connectedCount = droneApiStore.size
            if (connectedCount > 0) {
                if (connectedCount == 1) {
                    notifBuilder.setContentText("1 connected app")
                } else {
                    notifBuilder.setContentText("$connectedCount connected apps")
                }
            }
            val notification = notifBuilder.build()
            startForeground(FOREGROUND_ID, notification)
        }
    }

    @TargetApi(Build.VERSION_CODES.O)
    private fun createNotificationChannel(chanId: String, name: CharSequence): String {
        Timber.d("createNotificationChannel(%s, %s)", chanId, name)
        val channel = NotificationChannel(chanId, name, NotificationManager.IMPORTANCE_NONE)
        channel.lightColor = Color.BLUE
        channel.lockscreenVisibility = Notification.VISIBILITY_PRIVATE
        val service = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        service.createNotificationChannel(channel)
        return chanId
    }

    override fun onDestroy() {
        super.onDestroy()
        Timber.d("Destroying %s", TAG)
        for (droneApi in droneApiStore.values) {
            droneApi.destroy()
        }
        droneApiStore.clear()
        for (droneMgr in droneManagers.values) {
            droneMgr.destroy()
        }
        droneManagers.clear()
        dpServices!!.destroy()
        stopForeground(true)

        //Disable this service. It'll be reenabled the next time its local client needs it.
        enableDroidPlannerService(applicationContext, false)
    }

    override fun onStartCommand(intent: Intent, flags: Int, startId: Int): Int {
        Timber.d("onStartCommand(%s)", intent)
        if (intent != null) {
            when (intent.action) {
                ACTION_RELEASE_API_INSTANCE -> {
                    val appId = intent.getStringExtra(EXTRA_API_INSTANCE_APP_ID)
                    releaseDroneApi(appId)
                }
            }
        }
        stopSelf()
        return START_NOT_STICKY
    }

    companion object {
        val TAG = DroidPlannerService::class.java.simpleName

        /**
         * Status bar notification id
         */
        private const val FOREGROUND_ID = 101

        /**
         * Set of actions to notify the local app's components of the service events.
         */
        const val ACTION_DRONE_CREATED = Utils.PACKAGE_NAME + ".ACTION_DRONE_CREATED"
        const val ACTION_DRONE_DESTROYED = Utils.PACKAGE_NAME + ".ACTION_DRONE_DESTROYED"
        const val ACTION_RELEASE_API_INSTANCE = Utils.PACKAGE_NAME + ".action.RELEASE_API_INSTANCE"
        const val EXTRA_API_INSTANCE_APP_ID = "extra_api_instance_app_id"

        /**
         * Toggles the DroidPlannerService component
         * @param context
         * @param enable
         */
        @JvmStatic
        fun enableDroidPlannerService(context: Context, enable: Boolean) {
            Timber.d("enableDroidPlannerService(%s)", enable)
            val serviceComp = ComponentName(context, DroidPlannerService::class.java)
            val newState = if (enable) PackageManager.COMPONENT_ENABLED_STATE_ENABLED else PackageManager.COMPONENT_ENABLED_STATE_DISABLED
            context.packageManager.setComponentEnabledSetting(serviceComp, newState, PackageManager.DONT_KILL_APP)
        }
    }
}
