package com.o3dr.android.client.utils

import android.content.Context
import timber.log.Timber.DebugTree
import com.o3dr.android.client.utils.LogToFileTree
import android.content.pm.PackageInfo
import android.content.pm.PackageManager
import android.util.Log
import com.o3dr.android.client.BuildConfig
import timber.log.Timber
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.PrintStream
import java.text.SimpleDateFormat
import java.util.*
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Timber Tree to log specific log levels to a file
 */
class LogToFileTree : DebugTree() {
    private val logQueue = LinkedBlockingQueue<String>()
    private var logOutputFile: PrintStream? = null
    private var dequeueThread: Thread? = null
    private val isRunning = AtomicBoolean(false)
    private val date = Date()

    override fun log(priority: Int, tag: String, message: String, t: Throwable) {
        super.log(priority, tag, message, t)
        if (isLoggableToFile(priority)) {
            val logOutput = getLogMessage(priority, tag, message)
            logQueue.add(logOutput)
        }
    }

    private fun isLoggableToFile(priority: Int): Boolean {
        return priority >= BuildConfig.LOG_FILE_LEVEL
    }

    private fun getLogMessage(priority: Int, tag: String, message: String): String {
        val priorityShort = getPriorityString(priority)
        date.time = System.currentTimeMillis()
        return String.format("%s %s/%s : %s", LOG_DATE_FORMAT.format(date), priorityShort, tag, message)
    }

    private fun getPriorityString(priority: Int): String {
        var priorityString: String? = null
        priorityString = when (priority) {
            Log.ASSERT -> "ASSERT"
            Log.ERROR -> "E"
            Log.WARN -> "W"
            Log.INFO -> "I"
            Log.DEBUG -> "D"
            Log.VERBOSE -> "V"
            else -> ""
        }
        return priorityString
    }

    fun createFileStartLogging(context: Context) {
        if (dequeueThread != null && dequeueThread!!.isAlive) {
            stopLoggingThread()
        }
        dequeueThread = Thread {
            val pInfo: PackageInfo
            var version: String? = ""
            try {
                pInfo = context.packageManager.getPackageInfo(context.packageName, 0)
                version = pInfo.versionName
            } catch (e: PackageManager.NameNotFoundException) {
                Timber.w("Failed to get package info")
            }
            val rootDir = context.getExternalFilesDir(null)
            val dir = File(rootDir, "/log_cat/")
            dir.mkdirs()
            val fileName = String.format("%s_%s.log", version, FILE_DATE_FORMAT.format(Date()))
            val logFile = File(dir, fileName)
            try {
                logOutputFile = PrintStream(FileOutputStream(logFile, true))
                while (isRunning.get()) {
                    try {
                        val message = logQueue.take()
                        logOutputFile!!.println(message)
                    } catch (e: InterruptedException) {
                        Timber.w("Failed to receive message from logQueue")
                    }
                }
            } catch (e: IOException) {
                Timber.w("Failed to open file")
            } finally {
                isRunning.set(false)
                if (logOutputFile != null) {
                    logOutputFile!!.close()
                }
            }
        }
        isRunning.set(true)
        dequeueThread!!.start()
    }

    fun stopLoggingThread() {
        if (dequeueThread != null) {
            isRunning.set(false)
            dequeueThread!!.interrupt()
            dequeueThread = null
        }
    }

    companion object {
        private val LOG_DATE_FORMAT = SimpleDateFormat("MM-dd HH:mm:ss.SSS", Locale.US)
        private val FILE_DATE_FORMAT = SimpleDateFormat("yyyy_MM_dd_HH_mm", Locale.US)
    }
}
