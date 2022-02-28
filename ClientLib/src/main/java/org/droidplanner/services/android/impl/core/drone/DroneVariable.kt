package org.droidplanner.services.android.impl.core.drone

import android.os.Handler
import android.os.RemoteException
import com.o3dr.services.android.lib.model.ICommandListener
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import timber.log.Timber

open class DroneVariable<T : MavLinkDrone?>(protected var myDrone: T) {
    /**
     * Convenience method to post a success event to the listener.
     *
     * @param handler  Use to dispatch the event
     * @param listener To whom the event is dispatched.
     */
    protected fun postSuccessEvent(handler: Handler?, listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onSuccess()
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                }
            })
        }
    }

    /**
     * Convenience method to post an error event to the listener.
     *
     * @param handler  Use to dispatch the event
     * @param listener To whom the event is dispatched.
     * @param error    Execution error.
     */
    protected fun postErrorEvent(handler: Handler?, listener: ICommandListener?, error: Int) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onError(error)
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                }
            })
        }
    }

    /**
     * Convenience method to post a timeout event to the listener.
     *
     * @param handler  Use to dispatch the event
     * @param listener To whom the event is dispatched.
     */
    protected fun postTimeoutEvent(handler: Handler?, listener: ICommandListener?) {
        if (handler != null && listener != null) {
            handler.post(Runnable {
                try {
                    listener.onTimeout()
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                }
            })
        }
    }

    companion object {
        var UNSIGNED_BYTE_MIN_VALUE = 0
        var UNSIGNED_BYTE_MAX_VALUE = 255
        fun validateToUnsignedByteRange(id: Int): Short {
            require(!(id < UNSIGNED_BYTE_MIN_VALUE || id > UNSIGNED_BYTE_MAX_VALUE)) { "Value is outside of the range of an sysid/compid byte: $id" }
            return id.toShort()
        }
    }
}
