package org.droidplanner.services.android.impl.api

import android.os.Bundle
import android.os.RemoteException
import android.util.Log
import com.o3dr.android.client.BuildConfig
import com.o3dr.services.android.lib.gcs.event.GCSEvent
import com.o3dr.services.android.lib.model.IApiListener
import com.o3dr.services.android.lib.model.IDroidPlannerService
import com.o3dr.services.android.lib.model.IDroneApi
import com.o3dr.services.android.lib.util.version.VersionUtils.getCoreLibVersion
import java.util.*

private val TAG = DPServices::class.java.simpleName

/**
 * Created by fhuya on 11/3/14.
 */
internal class DPServices(private var serviceRef: DroidPlannerService?) : IDroidPlannerService.Stub() {
    fun destroy() {
        serviceRef = null
    }

    @Throws(RemoteException::class)
    override fun getServiceVersionCode(): Int {
        return BuildConfig.VERSION_CODE
    }

    @Throws(RemoteException::class)
    override fun getApiVersionCode(): Int {
        return getCoreLibVersion(serviceRef!!.applicationContext)
    }

    @Throws(RemoteException::class)
    override fun registerDroneApi(listener: IApiListener, appId: String): IDroneApi {
        return serviceRef!!.registerDroneApi(listener, appId)
    }

    @Throws(RemoteException::class)
    override fun getConnectedApps(requesterId: String): Array<Bundle> {
        Log.d(TAG, "List of connected apps request from $requesterId")
        val appsInfo: MutableList<Bundle> = ArrayList()
        for (droneApi in serviceRef!!.droneApiStore.values) {
            if (droneApi.isConnected) {
                val droneManager = droneApi.droneManager
                if (droneManager != null) {
                    val droneParams = droneApi.droneManager?.connectionParameter
                    val sanitizedParams = droneParams?.clone()

                    appsInfo.add(Bundle().apply {
                        putString(GCSEvent.EXTRA_APP_ID, droneApi.ownerId)
                        putParcelable(GCSEvent.EXTRA_VEHICLE_CONNECTION_PARAMETER, sanitizedParams)
                    })
                }
            }
        }
        return appsInfo.toTypedArray()
    }

    @Throws(RemoteException::class)
    override fun releaseDroneApi(dpApi: IDroneApi) {
        Log.d(TAG, "Releasing acquired drone api handle.")
        if (dpApi is DroneApi) {
            serviceRef?.releaseDroneApi(dpApi.ownerId)
        }
    }
}
