package org.droidplanner.services.android.impl.api

import android.os.Binder
import org.droidplanner.services.android.impl.api.DroidPlannerService
import org.droidplanner.services.android.impl.api.DroneApi
import java.util.ArrayList

/**
 * Created by fhuya on 11/3/14.
 */
class DroneAccess internal constructor(private val serviceRef: DroidPlannerService) : Binder() {
    val droneApiList: List<DroneApi>
        get() = ArrayList(serviceRef.droneApiStore.values)
}
