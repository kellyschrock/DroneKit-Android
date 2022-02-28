package com.o3dr.android.client.utils.data.tlog

/**
 * Filter class for TLog parser to allow the caller to determine the criteria for the list
 * of returned events.
 */
interface TLogParserFilter {
    /**
     * This method is called when an event is parsed to determine whether the caller wants this result
     * in the returned list.
     *
     * @param event [com.o3dr.android.client.utils.data.tlog.TLogParser.Event]
     * @return whether this event should be accepted based off criteria
     */
    fun includeEvent(event: TLogParser.Event?): Boolean

    /**
     * This method determines whether to iterate through the TLog file.
     *
     * @return whether to continue iterating or stop and send results
     */
    fun shouldIterate(): Boolean
}
