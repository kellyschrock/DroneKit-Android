package org.droidplanner.services.android.impl.utils.file

import android.content.Context
import android.os.Environment
import org.droidplanner.services.android.impl.utils.file.DirectoryPath

object DirectoryPath {
    /**
     * Main path used to store private data files related to the program
     *
     * @return Path to 3DR Services private data folder in external storage
     */
    @JvmStatic
    fun getPrivateDataPath(context: Context): String {
        val dataDir = context.getExternalFilesDir(null)
        return dataDir.absolutePath
    }

    /**
     * Main path used to store public data files related to the app.
     * @param context application context
     * @return Path to 3DR Services public data directory.
     */
	@JvmStatic
	fun getPublicDataPath(context: Context?): String {
        val root = Environment.getExternalStorageDirectory().path
        return "$root/3DRServices/"
    }

    /**
     * Storage folder for user camera description files
     */
    @JvmStatic
    fun getCameraInfoPath(context: Context?): String {
        return getPublicDataPath(context) + "/CameraInfo/"
    }

    /**
     * Storage folder for stacktraces
     */
    @JvmStatic
    fun getCrashLogPath(context: Context): String {
        return getPrivateDataPath(context) + "/crash_log/"
    }
}
