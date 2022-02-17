package org.droidplanner.services.android.impl.core.srtm

import org.droidplanner.services.android.impl.core.srtm.Srtm.OnProgressListner
import java.io.File
import java.io.IOException
import java.util.*

class SrtmRegions(private val path: String) {
    private val regionMap: MutableMap<String, Int> = HashMap()

    /*
	 * Returns region name for a file
	 */
    @Throws(Exception::class)
    fun findRegion(fname: String, listner: OnProgressListner): String {
        if (regionMap.isEmpty()) {
            fillRegionData(listner)
        }
        val name = fname.replace(".hgt", "")
        if (regionMap.containsKey(name)) {
            return REGIONS[regionMap[name]!!]
        }
        throw Exception("Null Region")
    }

    @Throws(Exception::class)
    private fun fillRegionData(listner: OnProgressListner) {
        var region: String
        for (i in REGIONS.indices) {
            region = REGIONS[i]
            var indexPath = region
            indexPath = SrtmDownloader.getIndexPath(path) + indexPath
            val indexDir = File(indexPath)
            if (!indexDir.exists()) {
                indexDir.mkdirs()
            }
            indexPath += ".index.html"
            val indexFile = File(indexPath)
            if (!indexFile.exists()) {
                try {
                    SrtmDownloader(listner).downloadRegionIndex(i, path)
                } catch (e: IOException) {
                    // download error, try again with the next attempt
                    regionMap.clear()
                    throw Exception("Null Region")
                }
            }
            val scanner = Scanner(indexFile)
            while (scanner.hasNext()) {
                val line = scanner.next()
                if (line.contains("href=\"")) {
                    var index = line.indexOf(".hgt.zip") - 7
                    if (index >= 0) {
                        val srtm = line.substring(index, index + 7)
                        regionMap[srtm] = i
                    } else {
                        index = line.indexOf("hgt.zip") - 7
                        if (index >= 0) {
                            val srtm = line.substring(index, index + 7)
                            regionMap[srtm] = i
                        }
                    }
                }
            }
            scanner.close()
        }
    }

    companion object {
        @JvmField
		val REGIONS = arrayOf("Eurasia", "Africa", "Australia", "Islands", "North_America",
                "South_America")
    }
}
