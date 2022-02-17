package org.droidplanner.services.android.impl.core.srtm

import org.droidplanner.services.android.impl.core.srtm.Srtm.OnProgressListner
import org.droidplanner.services.android.impl.core.srtm.UnZip.unZipIt
import java.io.*
import java.net.URL

class SrtmDownloader(private val listener: OnProgressListner?) {
    @Throws(IOException::class)
    fun downloadRegionIndex(region: Int, srtmPath: String) {
        var regionIndex = SrtmRegions.REGIONS[region] + ".index.html"
        regionIndex = getIndexPath(srtmPath) + regionIndex
        val regionIndexFile = File(regionIndex)
        downloadFile(url + SrtmRegions.REGIONS[region] + "/", regionIndexFile)
    }

    @Throws(Exception::class)
    fun downloadSrtmFile(fname: String, path: String) {
        val output: File
        val region = SrtmRegions(path).findRegion(fname, listener!!)
        output = File("$path/$fname.zip")
        downloadSrtmFile(fname, output, region)
        unZipIt(fname, output)
        output.delete()
    }

    @Throws(IOException::class)
    private fun downloadSrtmFile(fname: String, output: File, region: String) {
        try {
            downloadFile(url + region + "/" + fname + ".zip", output)
        } catch (e: IOException) {
            downloadAlternativeSrtmFile(fname, output, region, e)
        }
    }

    @Throws(IOException::class)
    private fun downloadAlternativeSrtmFile(fname: String, output: File, region: String, e: IOException) {
        // fix SRTM 2.1 naming problem in North America
        if (fname.startsWith("N5") && region.equals("North_America", ignoreCase = true)) {
            downloadFile(url + region + "/" + fname.replace(".hgt", "hgt") + ".zip",
                    output)
        } else {
            throw e
        }
    }

    @Throws(IOException::class)
    private fun downloadFile(urlAddress: String, file: File) {
        val url = URL(urlAddress)
        val connection = url.openConnection()
        connection.connect()
        // this will be useful so that you can show a typical 0-100% progress
        // bar
        val fileLength = connection.contentLength.toLong()

        // download the file
        val input: InputStream = BufferedInputStream(url.openStream())
        val outputs = BufferedOutputStream(FileOutputStream(file))
        val data = ByteArray(2048)
        var total: Long = 0
        var count: Int
        while (input.read(data).also { count = it } != -1) {
            total += count.toLong()
            outputs.write(data, 0, count)
            callListener(file.name, (total * 100 / fileLength).toInt())
        }
        outputs.flush()
        outputs.close()
        input.close()
    }

    private fun callListener(filename: String, i: Int) {
        if (listener != null) {
            if (i >= 0) {
                listener.onProgress(filename, i)
            } else {
                listener.onProgress(filename, -1)
            }
        }
    }

    companion object {
        const val url = "http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/"
        fun getIndexPath(srtmPath: String): String {
            return "$srtmPath/Index/"
        }
    }
}
