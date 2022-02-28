package org.droidplanner.services.android.impl.utils.connection

import android.support.v4.util.Pair
import android.text.TextUtils
import com.jcraft.jsch.*
import timber.log.Timber
import java.io.*

/**
 * Created by Fredia Huya-Kouadio on 2/24/15.
 */
class SshConnection(private val host: String, private val username: String, private val password: String) {
    interface UploadListener {
        fun onUploaded(uploadFile: File?, uploadBytesCount: Long, totalBytesCount: Long)
        fun shouldContinueUpload(): Boolean
    }

    interface DownloadListener {
        fun onFileSizeCalculated(fileSize: Long)
        fun onDownloaded(downloadFile: String?, downloadBytesCount: Long)
    }

    private val jsch: JSch = JSch()

    //Try to connect with the password set.
    @get:Throws(JSchException::class)
    private val session: Session
        private get() {
            val session = jsch.getSession(username, host)

            //Try to connect with the password set.
            session.setConfig("StrictHostKeyChecking", "no")
            session.setPassword(password)
            session.connect(CONNECTION_TIMEOUT)
            return session
        }

    fun ping(): Boolean {
        var session: Session? = null
        return try {
            session = session
            true
        } catch (e: JSchException) {
            false
        } finally {
            if (session != null && session.isConnected) session.disconnect()
        }
    }

    @Throws(IOException::class)
    fun execute(command: String?): String? {
        if (TextUtils.isEmpty(command)) return null
        var session: Session? = null
        var execChannel: Channel? = null
        return try {
            session = session
            execChannel = session!!.openChannel(EXEC_CHANNEL_TYPE)
            (execChannel as ChannelExec?)!!.setCommand(command)
            execChannel.setInputStream(null)
            val `in` = execChannel.getInputStream()
            execChannel.connect(CONNECTION_TIMEOUT)
            val bufferSize = 1024
            val response = StringBuilder()
            val buffer = ByteArray(bufferSize)
            while (true) {
                while (`in`.available() > 0) {
                    val dataSize = `in`.read(buffer, 0, bufferSize)
                    if (dataSize < 0) break
                    response.append(String(buffer, 0, dataSize))
                }
                if (execChannel.isClosed()) {
                    if (`in`.available() > 0) continue
                    Timber.d("SSH command exit status: " + execChannel.getExitStatus())
                    break
                }
            }
            response.toString()
        } catch (e: JSchException) {
            throw IOException(e)
        } finally {
            if (execChannel != null && execChannel.isConnected) execChannel.disconnect()
            if (session != null && session.isConnected) session.disconnect()
        }
    }

    @Throws(IOException::class)
    fun executeWithExitCode(command: String?): Pair<Int, String>? {
        if (TextUtils.isEmpty(command)) return null
        var session: Session? = null
        var execChannel: Channel? = null
        return try {
            session = session
            execChannel = session!!.openChannel(EXEC_CHANNEL_TYPE)
            (execChannel as ChannelExec?)!!.setCommand(command)
            execChannel.setInputStream(null)
            val `in` = execChannel.getInputStream()
            execChannel.connect(CONNECTION_TIMEOUT)
            val bufferSize = 1024
            val response = StringBuilder()
            val buffer = ByteArray(bufferSize)
            while (true) {
                while (`in`.available() > 0) {
                    val dataSize = `in`.read(buffer, 0, bufferSize)
                    if (dataSize < 0) break
                    response.append(String(buffer, 0, dataSize))
                }
                if (execChannel.isClosed()) {
                    if (`in`.available() > 0) continue
                    Timber.d("SSH command exit status: " + execChannel.getExitStatus())
                    break
                }
            }
            Pair(execChannel.getExitStatus(), response.toString())
        } catch (e: JSchException) {
            throw IOException(e)
        } finally {
            if (execChannel != null && execChannel.isConnected) execChannel.disconnect()
            if (session != null && session.isConnected) session.disconnect()
        }
    }

    @JvmOverloads
    @Throws(IOException::class)
    fun downloadFile(
        localFile: String?,
        remoteFilePath: String?,
        listener: DownloadListener? = null
    ): Boolean {
        if (localFile == null || remoteFilePath == null) return false
        var session: Session? = null
        val execChannel: Channel? = null
        var fos: FileOutputStream? = null
        var out: OutputStream? = null
        var input: InputStream? = null
        try {
            var prefix: String? = null
            if (File(localFile).isDirectory) {
                prefix = localFile + File.separator
            }
            session = session

            // exec 'scp -f remoteFilePath' remotely
            val command = "scp -f $remoteFilePath"
            val channel = session!!.openChannel("exec")
            (channel as ChannelExec).setCommand(command)

            // get I/O streams for remote scp
            out = channel.getOutputStream()
            input = channel.getInputStream()
            channel.connect()
            val buf = ByteArray(1024)

            // send '\0'
            buf[0] = 0
            out.write(buf, 0, 1)
            out.flush()
            val c = checkAck(input)
            if (c != 'C'.toInt()) {
                return false
            }

            // read '0644 '
            input.read(buf, 0, 5)
            var fileSize = 0L
            while (true) {
                if (input.read(buf, 0, 1) < 0) {
                    // error
                    return false
                }
                if (buf[0].toChar() == ' ') break
                fileSize = fileSize * 10L + (buf[0].toChar() - '0') as Long
            }
            var file: String? = null
            var i = 0
            while (true) {
                input.read(buf, i, 1)
                if (buf[i] == 0x0a.toByte()) {
                    file = String(buf, 0, i)
                    break
                }
                i++
            }
            listener?.onFileSizeCalculated(fileSize)
            // send '\0'
            buf[0] = 0
            out.write(buf, 0, 1)
            out.flush()

            // read a content of localFile
            fos = FileOutputStream(if (prefix == null) localFile else prefix + file)
            var bytesToRead: Int
            var progress: Long = 0
            while (true) {
                bytesToRead = if (buf.size < fileSize) buf.size else fileSize.toInt()
                bytesToRead = input.read(buf, 0, bytesToRead)
                if (bytesToRead < 0) {
                    // error
                    return false
                }
                progress += bytesToRead.toLong()
                fos.write(buf, 0, bytesToRead)
                fileSize -= bytesToRead.toLong()
                if (fileSize == 0L) break
                listener?.onDownloaded(localFile, progress)
            }
            fos.close()
            fos = null
            if (checkAck(input) != 0) {
                return false
            }

            // send '\0'
            buf[0] = 0
            out.write(buf, 0, 1)
            out.flush()
            session.disconnect()
        } catch (e: JSchException) {
            throw IOException(e)
        } finally {
            fos?.close()
            out?.close()
            input?.close()
            if (execChannel != null && execChannel.isConnected) execChannel.disconnect()
            if (session != null && session.isConnected) session.disconnect()
        }
        return true
    }

    @Throws(IOException::class)
    fun uploadFile(localFile: File?, remoteFilePath: String, listener: UploadListener?): Boolean {
        if (localFile == null || !localFile.isFile || listener != null && !listener.shouldContinueUpload()) return false
        var session: Session? = null
        var execChannel: Channel? = null
        var fis: FileInputStream? = null
        var out: OutputStream? = null
        var `in`: InputStream? = null
        return try {
            session = session
            var command = "scp -t $remoteFilePath"
            execChannel = session!!.openChannel(EXEC_CHANNEL_TYPE)
            (execChannel as ChannelExec?)!!.setCommand(command)

            //Get I/O streams for remote scp
            out = execChannel.getOutputStream()
            `in` = execChannel.getInputStream()
            execChannel.connect(CONNECTION_TIMEOUT)
            if (checkAck(`in`) != 0) return false
            if (listener != null && !listener.shouldContinueUpload()) return false

            //Send "C0644 fileSize filename"
            val fileSize = localFile.length()
            command = """C0644 $fileSize ${localFile.name} """
            out.write(command.toByteArray())
            out.flush()
            if (checkAck(`in`) != 0) return false

            //Send local file content
            val bufferSize = 8192
            fis = FileInputStream(localFile)
            val buffer = ByteArray(bufferSize)
            var uploadedBytesCount: Long = 0
            while (true) {
                val len = fis.read(buffer, 0, bufferSize)
                if (len <= 0) break
                out.write(buffer, 0, len)
                uploadedBytesCount += len.toLong()
                if (listener != null) {
                    listener.onUploaded(localFile, uploadedBytesCount, fileSize)
                    if (!listener.shouldContinueUpload()) return false
                }
            }

            //Send '\0'
            out.write(0)
            out.flush()
            if (checkAck(`in`) != 0) false else true
        } catch (e: JSchException) {
            throw IOException(e)
        } finally {
            fis?.close()
            out?.close()
            `in`?.close()
            if (execChannel != null && execChannel.isConnected) execChannel.disconnect()
            if (session != null && session.isConnected) session.disconnect()
        }
    }

    companion object {
        private val TAG = SshConnection::class.java.simpleName
        private const val CONNECTION_TIMEOUT = 15000 //ms
        private const val EXEC_CHANNEL_TYPE = "exec"
        @Throws(IOException::class)
        private fun checkAck(`in`: InputStream?): Int {
            val result = `in`!!.read()
            // result may be 0 for success,
            //          1 for error,
            //          2 for fatal error,
            //              -1
            if (result == 1 || result == 2) {
                //Log the error
                val errorMsg = StringBuilder()
                var character: Int
                do {
                    character = `in`.read()
                    errorMsg.append(character.toChar())
                } while (character != '\n'.toInt())
                if (errorMsg.length > 0) Timber.e(errorMsg.toString())
            }
            return result
        }
    }
}
