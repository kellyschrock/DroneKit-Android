package com.o3dr.android.client.utils.data.tlog

/**
 * Callback for asynchronous TLog parser.
 */
interface TLogParserCallback {
    /**
     * Callback for successful retrieval of one or more Event.
     *
     * @param events [com.o3dr.android.client.utils.data.tlog.TLogParser.Event]
     */
    fun onResult(events: List<TLogParser.Event?>?)

    /**
     * Callback for unsuccessful retrieval of Events.
     * [java.util.NoSuchElementException] is returned when the tlogs contain no Events
     * matching the criteria.
     *
     * @param e
     */
    fun onFailed(e: Exception?)
}
