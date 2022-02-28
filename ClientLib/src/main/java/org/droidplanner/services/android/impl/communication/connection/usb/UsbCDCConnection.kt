package org.droidplanner.services.android.impl.communication.connection.usb

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.util.Log
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus
import com.o3dr.services.android.lib.gcs.link.LinkConnectionStatus.Companion.newFailedConnectionStatus
import org.droidplanner.services.android.impl.communication.connection.usb.UsbConnection.UsbConnectionImpl
import timber.log.Timber
import java.io.IOException
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference

private val TAG = UsbCDCConnection::class.java.simpleName
private const val ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION"
private val intentFilter = IntentFilter(ACTION_USB_PERMISSION)

internal class UsbCDCConnection(context: Context?, parentConn: UsbConnection?, baudRate: Int) : UsbConnectionImpl(context!!, parentConn!!, baudRate) {
    private val serialDriverRef = AtomicReference<UsbSerialDriver?>()
    private val usbPermissionIntent: PendingIntent = PendingIntent.getBroadcast(context, 0, Intent(ACTION_USB_PERMISSION), 0)

    private val broadcastReceiver: BroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val action = intent.action
            if (ACTION_USB_PERMISSION == action) {
                removeWatchdog()
                val device = intent.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
                if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                    if (device != null) {
                        //call method to set up device communication
                        try {
                            openUsbDevice(device)
                        } catch (e: IOException) {
                            Log.e(TAG, e.message, e)
                        }
                    } else {
                        val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.LINK_UNAVAILABLE, "Unable to access usb device.")
                        onUsbConnectionStatus(connectionStatus)
                    }
                } else {
                    Log.d(TAG, "permission denied for device $device")
                    val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.PERMISSION_DENIED, "USB Permission denied.")
                    onUsbConnectionStatus(connectionStatus)
                }
            }
        }
    }

    private val permissionWatchdog = Runnable {
        Log.d(TAG, "Permission request timeout.")
        val connectionStatus = newFailedConnectionStatus(LinkConnectionStatus.TIMEOUT, "Unable to get usb access.")
        onUsbConnectionStatus(connectionStatus)
        removeWatchdog()
    }

    private var scheduler: ScheduledExecutorService? = null

    private fun registerUsbPermissionBroadcastReceiver() {
        context.registerReceiver(broadcastReceiver, intentFilter)
    }

    private fun unregisterUsbPermissionBroadcastReceiver() {
        try {
            context.unregisterReceiver(broadcastReceiver)
        } catch (e: IllegalArgumentException) {
            Timber.e(e, "Receiver was not registered.")
        }
    }

    private fun removeWatchdog() {
        scheduler?.shutdown()
        scheduler = null
    }

    @Throws(IOException::class)
    override fun openUsbConnection() {
        registerUsbPermissionBroadcastReceiver()

        // Get UsbManager from Android.
        val manager = context.getSystemService(Context.USB_SERVICE) as UsbManager

        //Get the list of available devices
        val availableDevices = UsbSerialProber.getAvailableSupportedDevices(manager)
        if (availableDevices.isEmpty()) {
            Log.d(TAG, "No Devices found")
            throw IOException("No Devices found")
        }

        //Pick the first device
        val device = availableDevices[0]
        if (manager.hasPermission(device)) {
            openUsbDevice(device)
        } else {
            removeWatchdog()
            scheduler = Executors.newSingleThreadScheduledExecutor()
            scheduler?.schedule(permissionWatchdog, 15, TimeUnit.SECONDS)
            Log.d(TAG, "Requesting permission to access usb device " + device.deviceName)
            manager.requestPermission(device, usbPermissionIntent)
        }
    }

    @Throws(IOException::class)
    private fun openUsbDevice(device: UsbDevice) {
        // Get UsbManager from Android.
        val manager = context.getSystemService(Context.USB_SERVICE) as UsbManager

        // Find the first available driver.
        val serialDriver = UsbSerialProber.openUsbDevice(manager, device)
        if (serialDriver == null) {
            Log.d(TAG, "No Devices found")
            throw IOException("No Devices found")
        } else {
            Log.d(TAG, "Opening using Baud rate $baudRate")
            try {
                serialDriver.open()
                serialDriver.setParameters(baudRate, 8, UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE)
                serialDriverRef.set(serialDriver)
                onUsbConnectionOpened()
            } catch (e: IOException) {
                Log.e(TAG, "Error setting up device: " + e.message, e)
                try {
                    serialDriver.close()
                } catch (e2: IOException) {
                    // Ignore.
                }
            }
        }
    }

    @Throws(IOException::class)
    override fun readDataBlock(readData: ByteArray?): Int {
        // Read data from driver. This call will return up to readData.length bytes.
        // If no data is received it will timeout after 200ms (as set by parameter 2)
        val serialDriver = serialDriverRef.get() ?: throw IOException("Device is unavailable.")
        var iavailable = 0
        iavailable = try {
            serialDriver.read(readData, 200)
        } catch (e: NullPointerException) {
            val errorMsg = """
                Error Reading: ${e.message}
                Assuming inaccessible USB device.  Closing connection.
                """.trimIndent()
            Log.e(TAG, errorMsg, e)
            throw IOException(errorMsg, e)
        }
        if (iavailable == 0) iavailable = -1
        return iavailable
    }

    override fun sendBuffer(buffer: ByteArray?) {
        // Write data to driver. This call should write buffer.length bytes
        // if data cant be sent , then it will timeout in 500ms (as set by
        // parameter 2)
        val serialDriver = serialDriverRef.get()
        if (serialDriver != null) {
            try {
                serialDriver.write(buffer, 500)
            } catch (e: IOException) {
                Log.e(TAG, "Error Sending: " + e.message, e)
            }
        }
    }

    @Throws(IOException::class)
    override fun closeUsbConnection() {
        unregisterUsbPermissionBroadcastReceiver()
        val serialDriver = serialDriverRef.getAndSet(null)
        if (serialDriver != null) {
            try {
                serialDriver.close()
            } catch (e: IOException) {
                Log.e(TAG, e.message, e)
            }
        }
    }

    override fun toString(): String = TAG
}
