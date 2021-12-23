package com.o3dr.android.client

import android.content.Context
import android.os.RemoteException
import com.o3dr.services.android.lib.drone.connection.ConnectionResult
import com.o3dr.services.android.lib.model.IApiListener
import com.o3dr.services.android.lib.util.version.VersionUtils.getCoreLibVersion

/**
 * Created by fhuya on 12/15/14.
 */
class DroneApiListener(private val context: Context) : IApiListener.Stub() {
    @Throws(RemoteException::class)
    override fun onConnectionFailed(connectionResult: ConnectionResult) {
    }

    @Throws(RemoteException::class)
    override fun getClientVersionCode(): Int {
        return BuildConfig.VERSION_CODE
    }

    override fun getApiVersionCode(): Int {
        return getCoreLibVersion(context)
    }
}
