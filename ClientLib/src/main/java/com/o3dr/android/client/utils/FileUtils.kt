package com.o3dr.android.client.utils

import android.content.Context
import org.droidplanner.services.android.impl.utils.file.DirectoryPath
import java.io.File
import java.io.FileNotFoundException
import java.io.FileOutputStream
import java.io.FilenameFilter
import java.text.SimpleDateFormat
import java.util.*
import kotlin.Throws

object FileUtils {
    const val CAMERA_FILENAME_EXT = ".xml"
    val timestampFormatter = SimpleDateFormat("yyyy_MM_dd_HH_mm_ss", Locale.US)

    @JvmStatic
    fun getCameraInfoFileList(context: Context?): Array<File?> {
        val filter = FilenameFilter { dir, filename -> filename.contains(CAMERA_FILENAME_EXT) }
        return getFileList(DirectoryPath.getCameraInfoPath(context), filter)
    }

    private fun getFileList(path: String, filter: FilenameFilter): Array<File?> {
        val mPath = File(path)
        return if (!mPath.exists()) arrayOfNulls(0) else mPath.listFiles(filter)
    }

    @JvmStatic
    @Throws(FileNotFoundException::class)
    fun getExceptionFileStream(context: Context?): FileOutputStream {
        val myDir = File(DirectoryPath.getCrashLogPath(context!!))
        if (!myDir.exists()) myDir.mkdirs()
        val file = File(myDir, "$timeStamp.log")
        if (file.exists()) file.delete()
        return FileOutputStream(file)
    }

    /**
     * Timestamp for logs in the Mission Planner Format
     */
    fun getTimeStamp(timestamp: Long): String {
        return timestampFormatter.format(Date(timestamp))
    }

    private val timeStamp: String
        private get() = getTimeStamp(System.currentTimeMillis())
}
