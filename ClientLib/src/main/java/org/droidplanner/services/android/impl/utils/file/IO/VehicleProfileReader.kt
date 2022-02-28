package org.droidplanner.services.android.impl.utils.file.IO

import android.content.Context
import android.util.Xml
import org.droidplanner.services.android.impl.core.drone.profiles.VehicleProfile
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.utils.file.AssetUtil.exists
import org.droidplanner.services.android.impl.utils.file.DirectoryPath.getPublicDataPath
import org.xmlpull.v1.XmlPullParser
import org.xmlpull.v1.XmlPullParserException
import java.io.*

object VehicleProfileReader {
    private const val VEHICLEPROFILE_PATH = "VehicleProfiles"

    // vehicle profile tags
    private const val TAG_METADATATYPE = "ParameterMetadataType"

    // default tags
    private const val TAG_DEFAULT = "Default"
    private const val ATTR_WPNAV_SPEED = "wpNavSpeed"
    private const val ATTR_MAX_ALTITUDE = "maxAltitude"
    private const val ATTR_TYPE = "type"

    /**
     * Load/aggregate profile from resources and file (if available) File will
     * override resource settings
     */
    fun load(context: Context, vehicleType: FirmwareType): VehicleProfile? {
        val fileName = "$vehicleType.xml"
        val path = VEHICLEPROFILE_PATH + File.separator + fileName
        return try {
            val newProfile = VehicleProfile()
            val file = File(getPublicDataPath(context) + path)
            if (file.exists()) {
                loadProfileFromFile(newProfile, file)
            } else {
                loadProfileFromResources(context, fileName, path, newProfile)
            }

            newProfile
        } catch (e: Exception) {
            e.printStackTrace()
            null
        }
    }

    @Throws(FileNotFoundException::class, XmlPullParserException::class, IOException::class)
    private fun loadProfileFromFile(newProfile: VehicleProfile, file: File) {
        val inputStream: InputStream = FileInputStream(file)
        open(inputStream, newProfile)
    }

    @Throws(IOException::class, XmlPullParserException::class)
    private fun loadProfileFromResources(context: Context, fileName: String,
                                         path: String, newProfile: VehicleProfile) {
        val assetManager = context.assets
        if (exists(assetManager, VEHICLEPROFILE_PATH, fileName)) {
            val inputStream = assetManager.open(path)
            open(inputStream, newProfile)
        }
    }

    @Throws(XmlPullParserException::class, IOException::class)
    private fun open(inputStream: InputStream, profile: VehicleProfile) {
        try {
            val parser = Xml.newPullParser()
            parser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false)
            parser.setInput(inputStream, null)
            parse(parser, profile)
        } finally {
            try {
                inputStream.close()
            } catch (e: IOException) { /* nop */
            }
        }
    }

    @Throws(XmlPullParserException::class, IOException::class)
    private fun parse(parser: XmlPullParser, profile: VehicleProfile) {
        var eventType = parser.eventType
        while (eventType != XmlPullParser.END_DOCUMENT) {
            val parserName = parser.name
            when (eventType) {
                XmlPullParser.START_TAG -> if (parserName == TAG_METADATATYPE) {
                    // set metadata type
                    val value = parser.getAttributeValue(null, ATTR_TYPE)
                    if (value != null) profile.parameterMetadataType = value
                } else if (parserName == TAG_DEFAULT) {
                    // set defaults
                    parseDefault(parser, profile.default)
                }
                XmlPullParser.END_TAG -> {}
            }
            eventType = parser.next()
        }
    }

    // parse Default
    private fun parseDefault(parser: XmlPullParser, default_: VehicleProfile.Default) {
        // wpNavSpeed
        var value = parser.getAttributeValue(null, ATTR_WPNAV_SPEED)
        if (value != null) default_.wpNavSpeed = parseInt(value)

        // maxAltitude
        value = parser.getAttributeValue(null, ATTR_MAX_ALTITUDE)
        if (value != null) default_.maxAltitude = parseInt(value)
    }

    private fun parseInt(str: String?): Int {
        return if (str == null) 0 else try {
            str.toInt()
        } catch (e: NumberFormatException) {
            0
        }
    }
}
