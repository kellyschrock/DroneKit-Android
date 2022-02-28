package com.o3dr.services.android.lib.model

/**
 * Basic command listener implementation.
 * Overrides the methods as needed to receive the command execution status notification.
 * Created by Fredia Huya-Kouadio on 6/24/15.
 */
open class SimpleCommandListener : AbstractCommandListener() {
    override fun onSuccess() {}
    override fun onError(executionError: Int) {}
    override fun onTimeout() {}
}
