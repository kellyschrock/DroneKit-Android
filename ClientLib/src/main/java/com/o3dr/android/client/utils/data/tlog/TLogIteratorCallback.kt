package com.o3dr.android.client.utils.data.tlog

/**
 * Callback for asynchronous TLog iterator.
 */
interface TLogIteratorCallback {
    /**
     * Callback for successful retrieval of next Event.
     *
     * @param event
     */
    fun onResult(event: TLogParser.Event?)

    /**
     * Callback for unsuccessful retrieval of next Event.
     * [java.util.NoSuchElementException] is returned when the tlogs contain no more Events
     * matching the criteria.
     *
     * @param e
     */
    fun onFailed(e: Exception?)
}
