package com.o3dr.android.client

import android.content.ComponentName
import android.content.Context
import android.content.ServiceConnection
import android.os.Bundle
import android.os.Handler
import android.os.IBinder
import android.os.IBinder.DeathRecipient
import android.os.RemoteException
import android.util.Log
import com.o3dr.android.client.interfaces.TowerListener
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter
import com.o3dr.services.android.lib.model.IDroidPlannerService
import com.o3dr.services.android.lib.model.IDroneApi
import timber.log.Timber
import java.util.concurrent.atomic.AtomicBoolean

private val TAG = ControlTower::class.java.simpleName

/**
 * Created by fhuya on 11/12/14.
 */
class ControlTower(private val context: Context) {
    private val binderDeathRecipient = DeathRecipient { notifyTowerDisconnected() }

    private val o3drServicesConnection: ServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName, service: IBinder) {
            Timber.d("onServiceConnected(%s, %s)", name, service)
            isServiceConnecting.set(false)
            o3drServices = IDroidPlannerService.Stub.asInterface(service)
            try {
                o3drServices?.asBinder()?.linkToDeath(binderDeathRecipient, 0)
                notifyTowerConnected()
            } catch (e: RemoteException) {
                Timber.e(e, e.message)
                notifyTowerDisconnected()
            }
        }

        override fun onServiceDisconnected(name: ComponentName) {
            Timber.d("onServiceDisconnected(%s)", name)
            isServiceConnecting.set(false)
            notifyTowerDisconnected()
        }
    }

    private val isServiceConnecting = AtomicBoolean(false)
    private val apiListener: DroneApiListener = DroneApiListener(context)
    private var towerListener: TowerListener? = null
    private var o3drServices: IDroidPlannerService? = null

    val isTowerConnected: Boolean
        get() = o3drServices != null && o3drServices!!.asBinder().pingBinder()

    fun notifyTowerConnected() {
        Timber.d("notifyTowerConnected()")
        towerListener?.onTowerConnected()
    }

    fun notifyTowerDisconnected() {
        towerListener?.onTowerDisconnected()
    }

    val connectedApps: Array<Bundle?>?
        get() {
            var connectedApps: Array<Bundle?>? = arrayOfNulls(0)
            if (isTowerConnected) {
                try {
                    connectedApps = o3drServices!!.getConnectedApps(applicationId)
                    if (connectedApps != null) {
                        val classLoader = ConnectionParameter::class.java.classLoader
                        for (appInfo in connectedApps) {
                            appInfo!!.classLoader = classLoader
                        }
                    }
                } catch (e: RemoteException) {
                    Log.e(TAG, e.message, e)
                }
            }
            return connectedApps
        }

    fun registerDrone(drone: Drone?, handler: Handler?) {
        if (drone == null) return
        check(isTowerConnected) { "Control Tower must be connected." }
        drone.init(this, handler)
        drone.start()
    }

    fun unregisterDrone(drone: Drone?) {
        drone?.destroy()
    }

    fun connect(listener: TowerListener?) {
        if (towerListener != null && (isServiceConnecting.get() || isTowerConnected)) {
            Timber.d("not connecting: alreadyConnecting=%s, isTowerConnected=%s", isServiceConnecting.get(), isTowerConnected)
            return
        }
        requireNotNull(listener) { "ServiceListener argument cannot be null." }
        towerListener = listener
        if (!isTowerConnected && !isServiceConnecting.get()) {
            val serviceIntent = ApiAvailability.getInstance().getAvailableServicesInstance(context)
            Timber.d("serviceIntent=%s", serviceIntent)
            Timber.d("Bind the service to %s", o3drServicesConnection)
            isServiceConnecting.set(context.bindService(serviceIntent, o3drServicesConnection,
                    Context.BIND_AUTO_CREATE))
        }
    }

    fun disconnect() {
        if (o3drServices != null) {
            o3drServices!!.asBinder().unlinkToDeath(binderDeathRecipient, 0)
            o3drServices = null
        }
        notifyTowerDisconnected()
        towerListener = null
        try {
            context.unbindService(o3drServicesConnection)
        } catch (e: Exception) {
            Log.e(TAG, "Error occurred while unbinding from 3DR Services.")
        }
    }

    @Throws(RemoteException::class)
    fun registerDroneApi(): IDroneApi {
        return o3drServices!!.registerDroneApi(apiListener, applicationId)
    }

    @Throws(RemoteException::class)
    fun releaseDroneApi(droneApi: IDroneApi?) {
        o3drServices?.releaseDroneApi(droneApi)
    }

    private val applicationId: String
        private get() = context.packageName
}
