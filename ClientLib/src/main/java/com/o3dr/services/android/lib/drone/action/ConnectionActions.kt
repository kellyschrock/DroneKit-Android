package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.util.Utils

/**
 * Contains builder methods use to generate connect or disconnect actions.
 */
object ConnectionActions {
    const val ACTION_CONNECT = Utils.PACKAGE_NAME + ".action.CONNECT"
    const val EXTRA_CONNECT_PARAMETER = "extra_connect_parameter"
    const val ACTION_DISCONNECT = Utils.PACKAGE_NAME + ".action.DISCONNECT"
}
