package org.droidplanner.services.android.impl.utils.video

import android.content.Context
import com.o3dr.android.client.utils.video.MediaCodecManager.NaluChunkListener
import android.media.MediaScannerConnection.OnScanCompletedListener
import timber.log.Timber
import com.o3dr.android.client.utils.video.NaluChunk
import kotlin.Throws
import com.googlecode.mp4parser.authoring.tracks.h264.H264TrackImpl
import com.googlecode.mp4parser.FileDataSourceImpl
import com.googlecode.mp4parser.authoring.builder.DefaultMp4Builder
import android.media.MediaScannerConnection
import android.os.Environment
import android.text.TextUtils
import com.googlecode.mp4parser.authoring.Movie
import java.io.*
import java.lang.NullPointerException
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/**
 * Created by Fredia Huya-Kouadio on 11/22/15.
 */
internal class StreamRecorder(private val context: Context) : NaluChunkListener {
    private val recordingFilename = AtomicReference<String?>()
    private val areParametersSet = AtomicBoolean(false)
    private val mediaRootDir: File = File(context.getExternalFilesDir(Environment.DIRECTORY_MOVIES), "stream")
    private val scanCompletedListener = OnScanCompletedListener { path, uri -> Timber.i("Media file %s was scanned successfully: %s", path, uri) }
    private var asyncExecutor: ExecutorService? = null
    private var h264Writer: BufferedOutputStream? = null
    fun getRecordingFilename(): String? {
        return recordingFilename.get()
    }

    fun startConverterThread() {
        if (asyncExecutor == null || asyncExecutor!!.isShutdown) {
            asyncExecutor = Executors.newSingleThreadExecutor()
        }
    }

    fun stopConverterThread() {
        if (asyncExecutor != null) asyncExecutor!!.shutdown()
    }

    val isRecordingEnabled: Boolean
        get() = !TextUtils.isEmpty(recordingFilename.get())

    fun enableRecording(mediaFilename: String?): Boolean {
        return if (!isRecordingEnabled) {
            areParametersSet.set(false)
            recordingFilename.set(mediaFilename)
            Timber.i("Enabling local recording to %s", mediaFilename)
            val h264File = File(mediaRootDir, mediaFilename)
            if (h264File.exists()) h264File.delete()
            try {
                h264Writer = BufferedOutputStream(FileOutputStream(h264File))
                true
            } catch (e: FileNotFoundException) {
                Timber.e(e, e.message)
                recordingFilename.set(null)
                false
            }
        } else {
            Timber.w("Video stream recording is already enabled")
            false
        }
    }

    fun disableRecording(): Boolean {
        if (isRecordingEnabled) {
            Timber.i("Disabling local recording")

            //Close the Buffered output stream
            if (h264Writer != null) {
                try {
                    h264Writer!!.close()
                } catch (e: IOException) {
                    Timber.e(e, e.message)
                } finally {
                    h264Writer = null

                    //Kickstart conversion of the h264 file to mp4.
                    convertToMp4(recordingFilename.get())
                    recordingFilename.set(null)
                }
            }
        }
        areParametersSet.set(false)
        return true
    }

    //TODO: Maybe put this on a background thread to avoid blocking on the write to file.
    override fun onNaluChunkUpdated(parametersSet: NaluChunk, dataChunk: NaluChunk) {
        if (isRecordingEnabled && h264Writer != null) {
            if (areParametersSet.get()) {
                try {
                    writeNaluChunk(h264Writer!!, dataChunk)
                } catch (e: IOException) {
                    Timber.e(e, e.message)
                }
            } else {
                try {
                    areParametersSet.set(writeNaluChunk(h264Writer!!, parametersSet))
                } catch (e: IOException) {
                    Timber.e(e, e.message)
                }
            }
        }
    }

    @Throws(IOException::class)
    private fun writeNaluChunk(bos: BufferedOutputStream, naluChunk: NaluChunk?): Boolean {
        if (naluChunk == null) return false
        val payloadCount: Int = naluChunk.payloads.size
        for (i in 0 until payloadCount) {
            val payload = naluChunk.payloads[i]!!
            if (payload.capacity() == 0) continue
            val dataLength = payload.position()
            val payloadData = payload.array()
            bos.write(payloadData, 0, dataLength)
        }
        return true
    }

    fun convertToMp4(filename: String?) {
        if (TextUtils.isEmpty(filename)) {
            Timber.w("Invalid media filename.")
            return
        }
        val rawMedia = File(mediaRootDir, filename)
        if (!rawMedia.exists()) {
            Timber.w("Media file doesn't exists.")
            return
        }
        if (rawMedia.length() == 0L) {
            Timber.w("Media file is empty.")
            return
        }
        asyncExecutor!!.execute {
            Timber.i("Starting h264 conversion process for media file %s.", filename)
            try {
                val h264Track = H264TrackImpl(FileDataSourceImpl(rawMedia))
                val movie = Movie()
                movie.addTrack(h264Track)
                val mp4File = DefaultMp4Builder().build(movie)
                val dstDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_MOVIES)
                val mp4Media = File(dstDir, "$filename.mp4")
                Timber.i("Generating the mp4 file @ %s", mp4Media.absolutePath)
                val fc = FileOutputStream(mp4Media).channel
                mp4File.writeContainer(fc)
                fc.close()

                //Delete the h264 file.
                Timber.i("Deleting raw h264 media file.")
                rawMedia.delete()

                //Add the generated file to the mediastore
                Timber.i("Adding the generated mp4 file to the media store.")
                MediaScannerConnection.scanFile(context, arrayOf(mp4Media.absolutePath), null, scanCompletedListener)
            } catch (e: IOException) {
                Timber.e(e, e.message)
            } catch (e: NullPointerException) {
                Timber.e(e, e.message)
            } catch (ex: Throwable) {
                Timber.e(ex, ex.message)
            }
        }
    }

    init {
        if (!mediaRootDir.exists()) {
            mediaRootDir.mkdirs()
        }
    }
}
