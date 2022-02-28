package com.o3dr.services.android.lib.util.version

import android.content.Context
import com.o3dr.android.client.R

/**
 * Created by fhuya on 11/12/14.
 */
object VersionUtils {
    /**
     * @param context
     * @return
     */
    @JvmStatic
    fun getCoreLibVersion(context: Context): Int {
        return context.resources.getInteger(R.integer.core_lib_version)
    }
}
