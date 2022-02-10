package org.droidplanner.services.android.impl.utils.file.IO

import android.content.Context
import android.util.Xml
import org.droidplanner.services.android.impl.core.drone.profiles.ParameterMetadata
import org.xmlpull.v1.XmlPullParser
import org.xmlpull.v1.XmlPullParserException
import timber.log.Timber
import java.io.IOException
import java.io.InputStream
import java.util.*

/**
 * Created by fhuya on 10/29/14.
 */
object ParameterMetadataLoader {
    private val TAG = ParameterMetadataLoader::class.java.simpleName
    private const val PARAMS_ASSET_PATH = "Parameters"
    private const val PARAMETERMETADATA_PATH = "Parameters/ParameterMetaData.xml"
    private const val METADATA_DISPLAYNAME = "DisplayName"
    private const val METADATA_DESCRIPTION = "Description"
    private const val METADATA_UNITS = "Units"
    private const val METADATA_VALUES = "Values"
    private const val METADATA_RANGE = "Range"
    @Throws(IOException::class, XmlPullParserException::class)
    fun load(context: Context, metadataType: String, metadata: MutableMap<String?, ParameterMetadata?>) {
        val assMan = context.assets
        val files = assMan.list(PARAMS_ASSET_PATH)
        if (files != null) {
            for (file in files) {
                val path = String.format("%s/%s", PARAMS_ASSET_PATH, file)
                Timber.d("Load metadata from %s for type %s", path, metadataType)
                val input = assMan.open(path)
                val map: MutableMap<String?, ParameterMetadata> = HashMap()
                open(input, metadataType, map)
                Timber.d("Read %d params", map.size)
                var added = 0
                for (k in map.keys) {
                    if (!metadata.containsKey(k)) {
                        metadata[k] = map[k]
                        ++added
                    }
                }
                Timber.d("Added %d params to map", added)
            }
        }
    }

    @Throws(XmlPullParserException::class, IOException::class)
    private fun open(inputStream: InputStream, metadataType: String, metadataMap: MutableMap<String?, ParameterMetadata>) {
        try {
            val parser = Xml.newPullParser()
            parser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false)
            parser.setInput(inputStream, null)
            parseMetadata(parser, metadataType, metadataMap)
        } finally {
            try {
                inputStream.close()
            } catch (e: IOException) { /* nop */
            }
        }
    }

    @Throws(XmlPullParserException::class, IOException::class)
    private fun parseMetadata(parser: XmlPullParser, metadataType: String, metadataMap: MutableMap<String?, ParameterMetadata>) {
        var name: String
        var parsing = false
        var metadata: ParameterMetadata? = null
        var eventType = parser.eventType
        while (eventType != XmlPullParser.END_DOCUMENT) {
            when (eventType) {
                XmlPullParser.START_TAG -> {
                    name = parser.name
                    // name == metadataType: start collecting metadata(s)
                    // metadata == null: create new metadata w/ name
                    // metadata != null: add to metadata as property
                    if (metadataType == name) {
                        parsing = true
                    } else if (parsing) {
                        if (metadata == null) {
                            metadata = ParameterMetadata()
                            metadata.name = name
                        } else {
                            addMetaDataProperty(metadata, name, parser.nextText())
                        }
                    }
                }
                XmlPullParser.END_TAG -> {
                    name = parser.name
                    // name == metadataType: done
                    // name == metadata.name: add metadata to metadataMap
                    if (metadataType == name) {
                        return
                    } else if (metadata != null && metadata.name == name) {
                        metadataMap[metadata.name] = metadata
                        metadata = null
                    }
                }
            }
            eventType = parser.next()
        }
        // no metadata
    }

    private fun addMetaDataProperty(metaData: ParameterMetadata, name: String, text: String) {
        when (name) {
            METADATA_DISPLAYNAME -> metaData.displayName = text
            METADATA_DESCRIPTION -> metaData.description = text
            METADATA_UNITS -> metaData.units = text
            METADATA_RANGE -> metaData.range = text
            METADATA_VALUES -> metaData.values = text
        }
    }
}
