package org.droidplanner.services.android.impl.utils.file.IO

import android.content.Context
import com.o3dr.android.client.utils.FileUtils.getExceptionFileStream
import java.io.PrintStream
import java.lang.Exception

class ExceptionWriter(private val context: Context) {
    fun saveStackTraceToSD(exception: Throwable?) {
        exception ?: return

        try {
            val out = PrintStream(getExceptionFileStream(context))
            exception.printStackTrace(out)
            out.close()
        } catch (excep: Exception) {
            excep.printStackTrace()
        }
    }
}
