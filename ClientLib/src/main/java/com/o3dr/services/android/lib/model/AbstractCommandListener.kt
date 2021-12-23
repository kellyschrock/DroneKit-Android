package com.o3dr.services.android.lib.model

/**
 * Created by Fredia Huya-Kouadio on 7/5/15.
 */
abstract class AbstractCommandListener : ICommandListener.Stub() {
    abstract override fun onSuccess()
    abstract override fun onError(executionError: Int)
    abstract override fun onTimeout()
}
