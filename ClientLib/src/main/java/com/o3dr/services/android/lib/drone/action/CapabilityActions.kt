package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 7/15/15.
 */
object CapabilityActions {
    const val ACTION_CHECK_FEATURE_SUPPORT = Utils.PACKAGE_NAME + ".action.CHECK_FEATURE_SUPPORT"

    /**
     * Id of the feature whose support to check.
     */
    const val EXTRA_FEATURE_ID = "extra_feature_id"
}
