package org.droidplanner.services.android.impl.utils

import android.util.Log
import org.droidplanner.services.android.impl.core.model.Logger
import java.lang.Exception

/**
 * Android specific implementation for the {org.droidplanner.services.android.core.model.Logger}
 * interface.
 */
class AndroidLogger  // Only one instance is allowed.
private constructor() : Logger {
    override fun logVerbose(logTag: String, verbose: String) {
        if (verbose != null) {
            Log.v(logTag, verbose)
        }
    }

    override fun logDebug(logTag: String, debug: String) {
        if (debug != null) {
            Log.d(logTag, debug)
        }
    }

    override fun logInfo(logTag: String, info: String) {
        if (info != null) {
            Log.i(logTag, info)
        }
    }

    override fun logWarning(logTag: String, warning: String) {
        if (warning != null) {
            Log.w(logTag, warning)
        }
    }

    override fun logWarning(logTag: String, exception: Exception) {
        if (exception != null) {
            Log.w(logTag, exception)
        }
    }

    override fun logWarning(logTag: String, warning: String, exception: Exception) {
        if (warning != null && exception != null) {
            Log.w(logTag, warning, exception)
        }
    }

    override fun logErr(logTag: String, err: String) {
        if (err != null) {
            Log.e(logTag, err)
        }
    }

    override fun logErr(logTag: String, exception: Exception) {
        if (exception != null) {
            Log.e(logTag, exception.message, exception)
        }
    }

    override fun logErr(logTag: String, err: String, exception: Exception) {
        if (err != null && exception != null) {
            Log.e(logTag, err, exception)
        }
    }

    companion object {
        val logger: Logger = AndroidLogger()
    }
}
