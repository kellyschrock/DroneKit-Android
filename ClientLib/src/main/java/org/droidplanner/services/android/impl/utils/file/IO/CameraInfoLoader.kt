package org.droidplanner.services.android.impl.utils.file.IO

import android.content.Context
import com.o3dr.android.client.utils.FileUtils.getCameraInfoFileList
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import java.io.FileInputStream
import java.io.FileNotFoundException
import java.io.IOException
import java.io.InputStream
import java.util.*

class CameraInfoLoader(private val context: Context) {
    private val filesInSdCard = HashMap<String, String>()
    private val filesInAssets = HashMap<String, String>()

    @Throws(Exception::class)
    fun openFile(file: String): CameraInfo {
        return if (filesInSdCard.containsKey(file)) {
            readSdCardFile(file)
        } else if (filesInAssets.containsKey(file)) {
            readAssetsFile(file)
        } else {
            throw FileNotFoundException()
        }
    }

    @Throws(Exception::class)
    private fun readSdCardFile(file: String): CameraInfo {
        val reader = CameraInfoReader()
        val inputStream: InputStream = FileInputStream(filesInSdCard[file])
        reader.openFile(inputStream)
        inputStream.close()
        return reader.cameraInfo
    }

    @Throws(Exception::class)
    private fun readAssetsFile(file: String): CameraInfo {
        val reader = CameraInfoReader()
        val inputStream = context.assets.open(filesInAssets[file])
        reader.openFile(inputStream)
        inputStream.close()
        return reader.cameraInfo
    }

    val cameraInfoList: List<String>
        get() {
            val avaliableCameras = ArrayList<String>()
            val cameraInfoListFromStorage = cameraInfoListFromStorage
            avaliableCameras.addAll(cameraInfoListFromStorage)
            val cameraInfoListFromAssets = cameraInfoListFromAssets
            avaliableCameras.addAll(cameraInfoListFromAssets)
            return avaliableCameras
        }

    private val cameraInfoListFromAssets: List<String>
        private get() = try {
            val list = context.assets.list(CAMERA_INFO_ASSESTS_FOLDER)
            filesInAssets.clear()
            for (string in list) {
                filesInAssets[string] = "$CAMERA_INFO_ASSESTS_FOLDER/$string"
            }
            Arrays.asList(*list)
        } catch (e: IOException) {
            ArrayList()
        }

    private val cameraInfoListFromStorage: List<String>
        private get() {
            val filesName: MutableList<String> = ArrayList()
            filesInSdCard.clear()
            val filesList = getCameraInfoFileList(context)
            if (filesList != null && filesList.isNotEmpty()) {
                for (file in filesList) {
                    val filename = file!!.name
                    filesName.add(filename)
                    filesInSdCard[filename] = file.absolutePath
                }
            }
            return filesName
        }

    companion object {
        private const val CAMERA_INFO_ASSESTS_FOLDER = "CameraInfo"
    }
}
