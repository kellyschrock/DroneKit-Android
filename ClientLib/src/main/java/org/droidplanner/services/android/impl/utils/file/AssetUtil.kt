package org.droidplanner.services.android.impl.utils.file

import kotlin.Throws
import android.content.res.AssetManager
import java.io.IOException

object AssetUtil {
    @JvmStatic
	@Throws(IOException::class)
    fun exists(assetManager: AssetManager, directory: String?, fileName: String): Boolean {
        val assets = assetManager.list(directory)
        for (asset in assets) if (asset == fileName) return true
        return false
    }
}
