package org.droidplanner.services.android.impl.core.model

import java.lang.Exception

/**
 * Defines a set of essential logging utilities.
 */
interface Logger {
    fun logVerbose(logTag: String, verbose: String)
    fun logDebug(logTag: String, debug: String)
    fun logInfo(logTag: String, info: String)
    fun logWarning(logTag: String, warning: String)
    fun logWarning(logTag: String, exception: Exception)
    fun logWarning(logTag: String, warning: String, exception: Exception)
    fun logErr(logTag: String, err: String)
    fun logErr(logTag: String, exception: Exception)
    fun logErr(logTag: String, err: String, exception: Exception)
}
