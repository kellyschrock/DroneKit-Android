package org.droidplanner.services.android.impl.core.srtm

import org.droidplanner.services.android.impl.core.srtm.Srtm.OnProgressListner
import java.io.BufferedInputStream
import java.io.File
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

class SrtmData(private val path: String) {
    private var srtmFile: File? = null
    private var s: BufferedInputStream? = null

    @Throws(Exception::class)
    fun load(lon: Double, lat: Double, listner: OnProgressListner): Int {
        val altitude: Int
        val fname = getName(lon, lat)
        setupFilePaths(fname)
        downloadSrtmFileIfNeeded(fname, listner)
        s = BufferedInputStream(FileInputStream(srtmFile))
        altitude = readHtgFile(s!!, lon, lat)
        s!!.close()
        return altitude
    }

    @Throws(Exception::class)
    private fun downloadSrtmFileIfNeeded(fname: String, listner: OnProgressListner) {
        if (!srtmFile!!.exists()) {
            SrtmDownloader(listner).downloadSrtmFile(fname, path)
        }
    }

    private fun setupFilePaths(fname: String) {
        srtmFile = File("$path/$fname")
    }

    @Throws(Exception::class)
    private fun readHtgFile(s: BufferedInputStream, lon: Double, lat: Double): Int {
        val buffer = ByteArray(2)
        val index = calculateFileIndex(lon, lat)
        skipToDataPositionInFile(index)
        s.read(buffer)
        return ByteBuffer.wrap(buffer).order(ByteOrder.BIG_ENDIAN).short.toInt()
    }

    @Throws(Exception::class)
    private fun skipToDataPositionInFile(index: Int) {
        if (s!!.skip(index.toLong()) != index.toLong()) {
            throw Exception("error when skipping")
        }
    }

    private fun calculateFileIndex(lon: Double, lat: Double): Int {
        val ai = Math.round(1200.0 * (lat - Math.floor(lat))).toInt()
        val aj = Math.round(1200.0 * (lon - Math.floor(lon))).toInt()
        return (aj + (1200 - ai) * 1201) * 2
    }

    companion object {
        fun getName(Dlon: Double, Dlat: Double): String {
            val lon = Math.floor(Dlon).toInt()
            val lat = Math.floor(Dlat).toInt()
            var dirlat = "N"
            if (lat < 0) {
                dirlat = "S"
            }
            var dirlon = "E"
            if (lon < 0) {
                dirlon = "W"
            }
            var st = Math.abs(lat).toString()
            while (st.length < 2) {
                st = "0$st"
            }
            var fname = dirlat + st
            st = Math.abs(lon).toString()
            while (st.length < 3) {
                st = "0$st"
            }
            fname = "$fname$dirlon$st.hgt"
            return fname
        }
    }
}
