package com.o3dr.android.client.utils.geotag

import com.o3dr.android.client.utils.geotag.GeoTagAsyncTask.GeoTagAlgorithm
import com.o3dr.android.client.utils.data.tlog.TLogParser
import java.io.File
import java.util.ArrayList
import java.util.HashMap

/**
 * Basic Algorithm that traverses backwards, matching Events to photo files
 */
internal class SimpleGeoTagAlgorithm : GeoTagAlgorithm {
    override fun match(events: List<TLogParser.Event>, photos: ArrayList<File>): HashMap<TLogParser.Event, File> {
        val matchedMap = HashMap<TLogParser.Event, File>()
        val eventsSize = events.size
        val photosSize = photos.size

        var i = eventsSize - 1
        var j = photosSize - 1
        while (i >= 0 && j >= 0) {
            matchedMap[events[i]] = photos[j]
            i--
            j--
        }
        return matchedMap
    }
}
