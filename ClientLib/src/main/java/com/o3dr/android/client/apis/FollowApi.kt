package com.o3dr.android.client.apis

import android.location.Location
import kotlin.jvm.JvmOverloads
import com.o3dr.services.android.lib.gcs.follow.FollowType
import com.o3dr.services.android.lib.gcs.follow.FollowLocationSource
import android.os.Bundle
import com.o3dr.android.client.Drone
import com.o3dr.services.android.lib.gcs.action.FollowMeActions
import com.o3dr.android.client.apis.FollowApi
import com.o3dr.services.android.lib.model.action.Action
import java.util.concurrent.ConcurrentHashMap

/**
 * Provides access to the Follow me api.
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
class FollowApi private constructor(private val drone: Drone) : Api() {
    /**
     * Enables follow-me if disabled.
     *
     * @param type follow-me mode to use.
     * @param source The location source to use
     */
    @JvmOverloads
    fun enableFollowMe(type: FollowType?, source: FollowLocationSource? = FollowLocationSource.INTERNAL) {
        val params = Bundle()
        params.putParcelable(FollowMeActions.EXTRA_FOLLOW_TYPE, type)
        params.putParcelable(FollowMeActions.EXTRA_LOCATION_SOURCE, source)
        drone.performAsyncAction(Action(FollowMeActions.ACTION_ENABLE_FOLLOW_ME, params))
    }

    /**
     * Updates the parameters for the currently enabled follow me mode.
     *
     * @param params Set of parameters for the current follow me mode.
     */
    fun updateFollowParams(params: Bundle?) {
        drone.performAsyncAction(Action(FollowMeActions.ACTION_UPDATE_FOLLOW_PARAMS, params))
    }

    /**
     * Disables follow me is enabled.
     */
    fun disableFollowMe() {
        drone.performAsyncAction(Action(FollowMeActions.ACTION_DISABLE_FOLLOW_ME))
    }

    /**
     * A new FollowLocation for the drone to follow.
     */
    fun updateLocation(loc: Location?) {
        val params = Bundle()
        params.putParcelable(FollowMeActions.EXTRA_LOCATION, loc)
        drone.performAsyncAction(Action(FollowMeActions.ACTION_NEW_EXTERNAL_LOCATION, params))
    }

    companion object {
        private val followApiCache = ConcurrentHashMap<Drone, FollowApi>()
        private val apiBuilder: Builder<FollowApi> = Builder { drone -> FollowApi(drone) }

        /**
         * Retrieves a FollowApi instance.
         *
         * @param drone target vehicle
         * @return a FollowApi instance.
         */
        fun getApi(drone: Drone?): FollowApi {
            return getApi(drone, followApiCache, apiBuilder)
        }
    }
}
