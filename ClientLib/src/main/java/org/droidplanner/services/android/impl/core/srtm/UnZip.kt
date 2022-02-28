package org.droidplanner.services.android.impl.core.srtm

import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.util.zip.ZipFile
import kotlin.Throws

object UnZip {
    @JvmStatic
    @Throws(IOException::class)
    fun unZipIt(fname: String, output: File) {
        val buffer = ByteArray(1024)
        val zip = ZipFile(output)
        val ze = zip.getEntry(fname)
        val zis = zip.getInputStream(ze)
        val newFile = File(output.parent + "/" + fname)
        val fos = FileOutputStream(newFile)
        var len: Int
        while (zis.read(buffer).also { len = it } > 0) {
            fos.write(buffer, 0, len)
        }
        fos.close()
        zis.close()
        zip.close()
    }
}
