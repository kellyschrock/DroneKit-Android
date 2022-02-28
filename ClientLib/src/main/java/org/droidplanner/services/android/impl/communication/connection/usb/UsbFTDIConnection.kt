package org.droidplanner.services.android.impl.communication.connection.usb

import android.content.Context
import android.util.Log
import com.ftdi.j2xx.D2xxManager
import com.ftdi.j2xx.D2xxManager.D2xxException
import com.ftdi.j2xx.FT_Device
import org.droidplanner.services.android.impl.communication.connection.usb.UsbConnection.UsbConnectionImpl
import java.io.IOException
import java.util.concurrent.atomic.AtomicReference
import kotlin.experimental.or

private val TAG = UsbFTDIConnection::class.java.simpleName
private const val LATENCY_TIMER: Byte = 32

internal class UsbFTDIConnection(
        context: Context?, parentConn: UsbConnection?, baudRate: Int)
: UsbConnectionImpl(context!!, parentConn!!, baudRate) {

    private val ftDevRef = AtomicReference<FT_Device?>()

    @Throws(IOException::class)
    override fun openUsbConnection() {
        var ftD2xx: D2xxManager? = null
        try {
            ftD2xx = D2xxManager.getInstance(context)
        } catch (ex: D2xxException) {
            logger.logErr(TAG, ex)
        }
        if (ftD2xx == null) {
            throw IOException("Unable to retrieve D2xxManager instance.")
        }
        val DevCount = ftD2xx.createDeviceInfoList(context)
        Log.d(TAG, "Found $DevCount ftdi devices.")
        if (DevCount < 1) {
            throw IOException("No Devices found")
        }
        var ftDev: FT_Device? = null
        try {
            // FIXME: The NPE is coming from the library. Investigate if it's
            // possible to fix there.
            ftDev = ftD2xx.openByIndex(context, 0)
        } catch (e: NullPointerException) {
            Log.e(TAG, e.message, e)
        } finally {
            if (ftDev == null) {
                throw IOException("No Devices found")
            }
        }
        Log.d(TAG, "Opening using Baud rate $baudRate")
        ftDev.setBitMode(0.toByte(), D2xxManager.FT_BITMODE_RESET)
        ftDev.setBaudRate(baudRate)
        ftDev.setDataCharacteristics(D2xxManager.FT_DATA_BITS_8, D2xxManager.FT_STOP_BITS_1,
                D2xxManager.FT_PARITY_NONE)
        ftDev.setFlowControl(D2xxManager.FT_FLOW_NONE, 0x00.toByte(), 0x00.toByte())
        ftDev.latencyTimer = LATENCY_TIMER
        ftDev.purge((D2xxManager.FT_PURGE_TX or D2xxManager.FT_PURGE_RX) as Byte)
        if (!ftDev.isOpen) {
            throw IOException("Unable to open usb device connection.")
        } else {
            Log.d(TAG, "COM open")
        }
        ftDevRef.set(ftDev)
        onUsbConnectionOpened()
    }

    @Throws(IOException::class)
    override fun readDataBlock(readData: ByteArray?): Int {
        val ftDev = ftDevRef.get()
        if (ftDev == null || !ftDev.isOpen) {
            throw IOException("Device is unavailable.")
        }
        var iavailable = ftDev.queueStatus
        if (iavailable > 0) {
            if (iavailable > 4096) iavailable = 4096
            try {
                ftDev.read(readData, iavailable)
            } catch (e: NullPointerException) {
                val errorMsg = """
                    Error Reading: ${e.message}
                    Assuming inaccessible USB device.  Closing connection.
                    """.trimIndent()
                Log.e(TAG, errorMsg, e)
                throw IOException(errorMsg, e)
            }
        }
        if (iavailable == 0) {
            iavailable = -1
        }
        return iavailable
    }

    override fun sendBuffer(buffer: ByteArray?) {
        val ftDev = ftDevRef.get()
        if (ftDev != null && ftDev.isOpen) {
            try {
                ftDev.write(buffer)
            } catch (e: Exception) {
                Log.e(TAG, "Error Sending: " + e.message, e)
            }
        }
    }

    @Throws(IOException::class)
    override fun closeUsbConnection() {
        val ftDev = ftDevRef.getAndSet(null)
        if (ftDev != null) {
            try {
                ftDev.close()
            } catch (e: Exception) {
                Log.e(TAG, e.message, e)
            }
        }
    }

    override fun toString(): String {
        return TAG
    }
}
