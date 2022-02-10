package org.droidplanner.services.android.impl.utils.file.IO

import android.util.Xml
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import org.xmlpull.v1.XmlPullParser
import org.xmlpull.v1.XmlPullParserException
import java.io.IOException
import java.io.InputStream
import java.lang.Exception
import kotlin.Throws

/**
 * Class to parse a Kml file, based on the code from
 * http://developer.android.com/training/basics/network-ops/xml.html
 *
 */
class CameraInfoReader {
    private var parser: XmlPullParser? = null
    val cameraInfo = CameraInfo()
    @Throws(Exception::class)
    fun openFile(inputStream: InputStream) {
        parse(inputStream)
        inputStream.close()
    }

    @Throws(XmlPullParserException::class, IOException::class)
    fun parse(input: InputStream?) {
        parser = Xml.newPullParser().apply {
            setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false)
            setInput(input, null)
            nextTag()
        }
        readCameraInfo()
    }

    @Throws(XmlPullParserException::class, IOException::class)
    private fun readCameraInfo() {
        parser ?: return

        parser!!.require(XmlPullParser.START_TAG, null, "cameraInfo")
        while (parser!!.next() != XmlPullParser.END_TAG) {
            if (parser!!.eventType != XmlPullParser.START_TAG) {
                continue
            }
            val name = parser!!.name
            // Starts by looking for the entry tag
            if (name == "SensorWidth") {
                cameraInfo.sensorWidth = readDouble("SensorWidth")
            } else if (name == "SensorHeight") {
                cameraInfo.sensorHeight = readDouble("SensorHeight")
            } else if (name == "SensorResolution") {
                cameraInfo.sensorResolution = readDouble("SensorResolution")
            } else if (name == "FocalLength") {
                cameraInfo.focalLength = readDouble("FocalLength")
            } else if (name == "Overlap") {
                cameraInfo.overlap = readDouble("Overlap")
            } else if (name == "Sidelap") {
                cameraInfo.sidelap = readDouble("Sidelap")
            } else if (name == "Name") {
                cameraInfo.name = readString("Name")
            } else if (name == "Orientation") {
                cameraInfo.isInLandscapeOrientation = readText() != "Portrait"
            } else {
                skip()
            }
        }
    }

    @Throws(IOException::class, XmlPullParserException::class)
    private fun readString(entry: String): String {
        parser!!.require(XmlPullParser.START_TAG, null, entry)
        val value = readText()
        parser!!.require(XmlPullParser.END_TAG, null, entry)
        return value
    }

    @Throws(IOException::class, XmlPullParserException::class)
    private fun readDouble(entry: String): Double {
        parser!!.require(XmlPullParser.START_TAG, null, entry)
        val value = java.lang.Double.valueOf(readText())
        parser!!.require(XmlPullParser.END_TAG, null, entry)
        return value
    }

    // For the tags title and summary, extracts their text values.
    @Throws(IOException::class, XmlPullParserException::class)
    private fun readText(): String {
        var result = ""
        if (parser!!.next() == XmlPullParser.TEXT) {
            result = parser!!.text
            parser!!.nextTag()
        }
        return result
    }

    // Skips tags the parser isn't interested in. Uses depth to handle
    // nested tags. i.e.,
    // if the next tag after a START_TAG isn't a matching END_TAG, it keeps
    // going until it
    // finds the matching END_TAG (as indicated by the value of "depth"
    // being 0).
    @Throws(XmlPullParserException::class, IOException::class)
    private fun skip() {
        check(parser!!.eventType == XmlPullParser.START_TAG)
        var depth = 1
        while (depth != 0) {
            when (parser!!.next()) {
                XmlPullParser.END_TAG -> depth--
                XmlPullParser.START_TAG -> depth++
            }
        }
    }
}
