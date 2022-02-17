package org.droidplanner.services.android.impl.core.srtm

class Srtm(directory: String?) {
    /**
     * Callback for progress reports
     */
    interface OnProgressListner {
        fun onProgress(filename: String?, percentage: Int)
    }

    private val srtmData: SrtmData
    private var listner: OnProgressListner? = null

    /**
     * Get SRTM elevation for geographic coordinate (WGS-84)
     *
     * Stores a cache of uncompressed SRTM data files at the default directory.
     * It need a Internet connection to fetch SRTM files if they are not in the
     * disk
     *
     * @return Above Sea Level (ASL) altitude in meters
     */
    fun getData(longitude: Double, latitude: Double): Int {
        return try {
            srtmData.load(longitude, latitude, listner!!)
        } catch (e: Exception) {
            e.printStackTrace()
            SRTM_NaN // SRTM NaN
        }
    }

    /**
     * If a file needs to be download this listener will be called periodically
     */
    fun setListner(listner: OnProgressListner?) {
        this.listner = listner
    }

    companion object {
        private const val SRTM_NaN = -32768
    }

    /**
     * @param directory
     * Cache directory
     */
    init {
        srtmData = SrtmData(directory!!)
    }
}
