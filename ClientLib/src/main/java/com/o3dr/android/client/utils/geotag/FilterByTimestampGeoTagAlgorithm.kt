package com.o3dr.android.client.utils.geotag

import android.util.Log
import com.MAVLink.ardupilotmega.msg_camera_feedback
import com.MAVLink.common.msg_named_value_int
import com.o3dr.android.client.utils.geotag.GeoTagAsyncTask.GeoTagAlgorithm
import com.o3dr.android.client.utils.data.tlog.TLogParser
import com.o3dr.android.client.utils.geotag.FilterByTimestampGeoTagAlgorithm
import java.io.File
import java.util.*

private val TAG = FilterByTimestampGeoTagAlgorithm::class.java.simpleName

/**
 * Created by Fredia Huya-Kouadio on 1/7/16.
 */
class FilterByTimestampGeoTagAlgorithm : GeoTagAlgorithm {
    /**
     * The list of events must start with an event whose mavlink message is a msg_named_value_int.
     * That msg_named_value_int contains the start time and label for the mission whose data we are geotagging.
     *
     * The mission start time, and end time (max event timestamp > start time) will be used to filter the media data set.
     *
     * @param events
     * @param photos
     * @return
     */
    override fun match(events: List<TLogParser.Event>, photos: ArrayList<File>): HashMap<TLogParser.Event, File>? {
        if (events.isEmpty() || photos.isEmpty()) return null
        val sortedEvents = TreeMap<Long, TLogParser.Event>()
        val filteredPhotos = TreeMap<Long, File>()

        //Find the msg_named_value_int event
        var startEvent: TLogParser.Event? = null
        for (event in events) {
            val eventMsg = event.mavLinkMessage

            //Only store the msg_camera_feedback events.
            if (eventMsg is msg_camera_feedback) {
                sortedEvents[event.timestamp] = event
            } else if (eventMsg is msg_named_value_int) {
                if (startEvent == null) {
                    startEvent = event
                } else {
                    Log.w(TAG, "There's more than one msg_named_value_int event in the event list.")
                    //Defaulting to the event with the earliest timestamp
                    if (startEvent.timestamp > event.timestamp) {
                        startEvent = event
                    }
                }
            }
        }

        if (startEvent == null) {
            //No start event was found. Aborting the process.
            return null
        }

        val startTime = startEvent.timestamp
        Log.i(TAG, "Filtering events for mission " + (startEvent.mavLinkMessage as msg_named_value_int).getName() + " with start time " + startTime)

        //Filter the events by timestamp
        val filteredEvents = sortedEvents.tailMap(startTime)
        if (filteredEvents.isEmpty()) {
            //No events survived the filtering
            return null
        }

        //Get the end time.
        val endTime = filteredEvents.lastKey()
        if (endTime <= startTime) {
            //Invalid time span.
            return null
        }

        //Get the timezone offset, and apply it to the photo modified time.
        val calendar = Calendar.getInstance()
        val timezoneOffset = (calendar[Calendar.ZONE_OFFSET] + calendar[Calendar.DST_OFFSET]).toLong() //Timezone offset in milliseconds

        //Filter and sort the media
        for (photo in photos) {
            //Get the file timestamp
            val modifiedTime = photo.lastModified()
            val updatedTime = modifiedTime + timezoneOffset
            if (updatedTime in startTime..endTime) {
                filteredPhotos[modifiedTime] = photo
            }
        }

        if (filteredPhotos.isEmpty()) {
            //No matching media
            return null
        }

        val result = HashMap<TLogParser.Event, File>()
        val eventCollection = ArrayList(sortedEvents.values)
        val eventSize = eventCollection.size
        val photoCollection = ArrayList(filteredPhotos.values)
        val photoSize = photoCollection.size
        var e = 0
        var p = 0
        while (e < eventSize && p < photoSize) {
            result[eventCollection[e]] = photoCollection[p]
            e++
            p++
        }
        return result
    }
}
