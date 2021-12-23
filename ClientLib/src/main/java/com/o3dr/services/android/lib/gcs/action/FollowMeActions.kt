package com.o3dr.services.android.lib.gcs.action

import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
object FollowMeActions {
    const val ACTION_ENABLE_FOLLOW_ME = Utils.PACKAGE_NAME + ".action.ENABLE_FOLLOW_ME"
    const val EXTRA_FOLLOW_TYPE = "extra_follow_type"
    const val ACTION_UPDATE_FOLLOW_PARAMS = Utils.PACKAGE_NAME + ".action.UPDATE_FOLLOW_PARAMS"
    const val ACTION_DISABLE_FOLLOW_ME = Utils.PACKAGE_NAME + ".action.DISABLE_FOLLOW_ME"
    const val ACTION_NEW_EXTERNAL_LOCATION = Utils.PACKAGE_NAME + ".action.NEW_EXTERNAL_LOCATION"
    const val EXTRA_LOCATION = "extra_location"
    const val EXTRA_LOCATION_SOURCE = "extra_location_source"
}
