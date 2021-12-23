package com.o3dr.android.client.utils.geotag

import com.o3dr.android.client.utils.data.tlog.TLogParser
import com.o3dr.android.client.utils.geotag.GeoTagAsyncTask.GeoTagAlgorithm
import kotlin.Throws
import android.media.ExifInterface
import com.MAVLink.ardupilotmega.msg_camera_feedback
import java.io.*
import java.lang.Exception
import java.lang.IllegalStateException
import java.util.ArrayList
import java.util.HashMap

/**
 * Created by Fredia Huya-Kouadio on 1/12/16.
 */
object GeoTagUtils {
    /**
     *
     * @param saveDir
     * @param events
     * @param photos
     * @param geoTagAlg
     * @param listener
     * @return
     */
    @JvmStatic
    fun geotag(saveDir: File, events: List<TLogParser.Event?>?, photos: ArrayList<File?>?, geoTagAlg: GeoTagAlgorithm, listener: GeoTagListener?): ResultObject {
        val resultObject = ResultObject()
        try {
            val eventsPhotos = HashMap<TLogParser.Event, File>()
            val geoTaggedFiles = HashMap<File, File>()
            val failedFiles = HashMap<File, Exception>()
            resultObject.setResult(eventsPhotos, geoTaggedFiles, failedFiles)
            if (!saveDir.mkdirs()) {
                resultObject.exception = IllegalStateException("Failed to create directory for images")
                return resultObject
            }
            val matchedPhotos = geoTagAlg.match(events, photos)
            if (matchedPhotos == null || matchedPhotos.isEmpty()) {
                resultObject.exception = IllegalStateException("Unable to match the media set for geotagging.")
                return resultObject
            }
            if (!hasEnoughMemory(saveDir, matchedPhotos.values)) {
                resultObject.exception = IllegalStateException("Insufficient external storage space.")
                return resultObject
            }
            val numTotal = matchedPhotos.size
            var numProcessed = 0
            for ((event, photo) in matchedPhotos) {
                val newFile = File(saveDir, photo.name)
                try {
                    copyFile(photo, newFile)
                    updateExif(event, newFile)
                    eventsPhotos[event] = newFile
                    geoTaggedFiles[photo] = newFile
                } catch (e: Exception) {
                    failedFiles[photo] = e
                }
                numProcessed++
                listener?.onProgress(numProcessed, numTotal)
            }
        } catch (e: Exception) {
            resultObject.exception = e
        }
        return resultObject
    }

    private fun hasEnoughMemory(file: File, photos: Collection<File>): Boolean {
        val freeBytes = file.usableSpace
        var bytesNeeded: Long = 0
        for (photo in photos) {
            bytesNeeded += photo.length()
        }
        return bytesNeeded <= freeBytes
    }

    @Throws(IOException::class)
    private fun copyFile(inputPath: File, outputPath: File) {
        val input: InputStream = FileInputStream(inputPath)
        val out: OutputStream = FileOutputStream(outputPath)
        val buffer = ByteArray(1024)
        var read: Int
        while (input.read(buffer).also { read = it } != -1) {
            out.write(buffer, 0, read)
        }
        input.close()

        // write the output file (You have now copied the file)
        out.flush()
        out.close()
    }

    @Throws(IOException::class)
    private fun updateExif(event: TLogParser.Event, photoFile: File) {
        val msg = event.mavLinkMessage as msg_camera_feedback
        val lat = msg.lat.toDouble() / 10000000
        val lng = msg.lng.toDouble() / 10000000
        val alt = msg.alt_msl.toString()
        val exifInterface = ExifInterface(photoFile.path)
        exifInterface.setAttribute(ExifInterface.TAG_GPS_LONGITUDE, convertLatLngToDMS(lng))
        exifInterface.setAttribute(ExifInterface.TAG_GPS_LATITUDE, convertLatLngToDMS(lat))
        exifInterface.setAttribute(ExifInterface.TAG_GPS_LATITUDE_REF, if (lat < 0) "S" else "N")
        exifInterface.setAttribute(ExifInterface.TAG_GPS_LONGITUDE_REF, if (lng < 0) "W" else "E")
        exifInterface.setAttribute(ExifInterface.TAG_GPS_ALTITUDE, alt)
        exifInterface.saveAttributes()
    }

    private fun convertLatLngToDMS(coord: Double): String {
        val dDegree = Math.abs(coord)
        val degree = dDegree.toInt()
        val dMinute = (dDegree - degree) * 60
        val minute = dMinute.toInt()
        val dSecond = (dMinute - minute) * 60
        val second = (dSecond * 1000).toInt()
        return String.format("%s/1,%s/1,%s/1000", degree, minute, second)
    }

    interface GeoTagListener {
        fun onProgress(numProcessed: Int, numTotal: Int)
    }

    class ResultObject {
        private var didSucceed = false
        var eventsPhotos: HashMap<TLogParser.Event, File>? = null
            private set
        var geoTaggedPhotos: HashMap<File, File>? = null
            private set
        var failedFiles: HashMap<File, Exception>? = null
            private set
        var exception: Exception? = null
            set(exception) {
                didSucceed = false
                field = exception
            }

        fun didSucceed(): Boolean {
            return didSucceed
        }

        fun setResult(eventsPhotos: HashMap<TLogParser.Event, File>?, geoTaggedPhotos: HashMap<File, File>?, failedFiles: HashMap<File, Exception>?) {
            didSucceed = true
            this.eventsPhotos = eventsPhotos
            this.geoTaggedPhotos = geoTaggedPhotos
            this.failedFiles = failedFiles
        }
    }
}
