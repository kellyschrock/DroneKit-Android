package org.droidplanner.services.android.impl.communication.connection.usb

import android.content.Context
import android.hardware.usb.UsbManager
import android.util.Log
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import org.droidplanner.services.android.impl.communication.connection.AndroidMavLinkConnection
import org.droidplanner.services.android.impl.core.MAVLink.connection.MavLinkConnectionTypes
import org.droidplanner.services.android.impl.utils.AndroidLogger
import java.io.IOException

private val TAG = UsbConnection::class.java.simpleName
private const val FTDI_DEVICE_VENDOR_ID = 0x0403

class UsbConnection(parentContext: Context?, private val baudRate: Int) : AndroidMavLinkConnection(parentContext!!) {
    private var usbConnection: UsbConnectionImpl? = null

    @Throws(IOException::class)
    override fun closeConnection() {
        if (usbConnection != null) {
            usbConnection!!.closeUsbConnection()
        }
    }

    override fun loadPreferences() {}
    @Throws(IOException::class)
    override fun openConnection() {
        if (usbConnection != null) {
            usbConnection = try {
                usbConnection!!.openUsbConnection()
                Log.d(TAG, "Reusing previous usb connection.")
                return
            } catch (e: IOException) {
                Log.e(TAG, "Previous usb connection is not usable.", e)
                null
            }
        }

        if (isFTDIdevice(context)) {
            val tmp: UsbConnectionImpl = UsbFTDIConnection(context, this, baudRate)
            try {
                tmp.openUsbConnection()

                // If the call above is successful, 'mUsbConnection' will be set.
                usbConnection = tmp
                Log.d(TAG, "Using FTDI usb connection.")
            } catch (e: IOException) {
                Log.d(TAG, "Unable to open a ftdi usb connection. Falling back to the open "
                        + "usb-library.", e)
            }
        }

        // Fallback
        if (usbConnection == null) {
            val tmp: UsbConnectionImpl = UsbCDCConnection(context, this, baudRate)

            // If an error happens here, let it propagate up the call chain since this is the fallback.
            tmp.openUsbConnection()
            usbConnection = tmp
            Log.d(TAG, "Using open-source usb connection.")
        }
    }

    @Throws(IOException::class)
    override fun readDataBlock(buffer: ByteArray): Int {
        if (usbConnection == null) {
            throw IOException("Uninitialized usb connection.")
        }

        return usbConnection!!.readDataBlock(buffer)
    }

    @Throws(IOException::class)
    override fun sendBuffer(buffer: ByteArray) {
        if (usbConnection == null) {
            throw IOException("Uninitialized usb connection.")
        }

        usbConnection!!.sendBuffer(buffer)
    }

    override fun getConnectionType(): Int {
        return MavLinkConnectionTypes.MAVLINK_CONNECTION_USB
    }

    override fun toString(): String {
        return if (usbConnection == null) {
            TAG
        } else usbConnection.toString()
    }

    internal abstract class UsbConnectionImpl protected constructor(protected val context: Context, private val parentConnection: UsbConnection, protected val baudRate: Int) {
        @JvmField
        protected val logger = AndroidLogger.getLogger()

        protected fun onUsbConnectionOpened() {
            parentConnection.onConnectionOpened()
        }

        protected fun onUsbConnectionStatus(connectionStatus: LinkConnectionStatus?) {
            parentConnection.onConnectionStatus(connectionStatus)
        }

        @Throws(IOException::class)
        abstract fun closeUsbConnection()
        @Throws(IOException::class)
        abstract fun openUsbConnection()
        @Throws(IOException::class)
        abstract fun readDataBlock(readData: ByteArray?): Int
        abstract fun sendBuffer(buffer: ByteArray?)
    }

    companion object {
        private fun isFTDIdevice(context: Context): Boolean {
            val manager = context.getSystemService(Context.USB_SERVICE) as UsbManager
            val deviceList = manager.deviceList
            if (deviceList == null || deviceList.isEmpty()) {
                return false
            }
            for ((_, value) in deviceList) {
                if (value.vendorId == FTDI_DEVICE_VENDOR_ID) {
                    return true
                }
            }
            return false
        }
    }
}
