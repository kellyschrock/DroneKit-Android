package org.droidplanner.services.android.impl.communication.connection

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothSocket
import kotlin.Throws
import android.bluetooth.BluetoothDevice
import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import org.droidplanner.services.android.impl.core.MAVLink.connection.MavLinkConnectionTypes
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.lang.IllegalArgumentException
import java.net.UnknownHostException
import java.util.*

class BluetoothConnection(parentContext: Context, private val bluetoothAddress: String)
: AndroidMavLinkConnection(parentContext) {

    private val bluetoothAdapter: BluetoothAdapter? = BluetoothAdapter.getDefaultAdapter()
    private var out: OutputStream? = null
    private var input: InputStream? = null
    private var bluetoothSocket: BluetoothSocket? = null

    @Throws(IOException::class)
    override fun openConnection() {
        Log.d(BLUE, "Connect")

        // Reset the bluetooth connection
        resetConnection()

        // Retrieve the stored device
        var device: BluetoothDevice? = null
        try {
            device = bluetoothAdapter!!.getRemoteDevice(bluetoothAddress)
        } catch (ex: IllegalArgumentException) {
            // invalid configuration (device may have been removed)
            // NOP fall through to 'no device'
        }

        // no device
        if (device == null) {
            device = findSerialBluetoothBoard()
        }

        Log.d(BLUE, "Trying to connect to device with address " + device.address)
        Log.d(BLUE, "BT Create Socket Call...")
        bluetoothSocket = device.createInsecureRfcommSocketToServiceRecord(UUID.fromString(UUID_SPP_DEVICE))
        Log.d(BLUE, "BT Cancel Discovery Call...")
        bluetoothAdapter?.cancelDiscovery()
        Log.d(BLUE, "BT Connect Call...")

        bluetoothSocket?.connect() // Here the IOException will rise on BT
        // protocol/handshake error.
        Log.d(BLUE, "## BT Connected ##")
        out = bluetoothSocket?.outputStream
        input = bluetoothSocket?.inputStream
        onConnectionOpened()
    }

    @SuppressLint("NewApi")
    @Throws(UnknownHostException::class)
    private fun findSerialBluetoothBoard(): BluetoothDevice {
        bluetoothAdapter?.bondedDevices?.let { pairedDevices ->
            if (pairedDevices.size > 0) {
                // Loop through paired devices
                for (device in pairedDevices) {
                    // Add the name and address to an array adapter to show in a ListView
                    Log.d(BLUE, device.name + " #" + device.address + "#")
                    val deviceUuids = device.uuids
                    if (deviceUuids != null && deviceUuids.size > 0) {
                        for (id in device.uuids) {
                            Log.d(BLUE, "id:$id")
                            if (id.toString().equals(UUID_SPP_DEVICE, ignoreCase = true)) {
                                Log.d(BLUE,
                                        ">> Selected: " + device.name + " Using: " + id.toString())
                                return device
                            }
                        }
                    }
                }
            }
        }

        // If there are paired devices
        throw UnknownHostException("No Bluetooth Device found")
    }

    @Throws(IOException::class)
    override fun readDataBlock(buffer: ByteArray): Int {
        input ?: throw IOException("No input stream")

        return input!!.read(buffer)
    }

    @Throws(IOException::class)
    override fun sendBuffer(buffer: ByteArray) {
        out?.write(buffer)
    }

    override fun getConnectionType(): Int = MavLinkConnectionTypes.MAVLINK_CONNECTION_BLUETOOTH

    @Throws(IOException::class)
    override fun closeConnection() {
        resetConnection()
        Log.d(BLUE, "## BT Closed ##")
    }

    @Throws(IOException::class)
    private fun resetConnection() {
        input?.close()
        input = null

        out?.close()
        out = null

        bluetoothSocket?.close()
        bluetoothSocket = null
    }

    override fun loadPreferences() {}

    companion object {
        private const val BLUE = "BLUETOOTH"
        private const val UUID_SPP_DEVICE = "00001101-0000-1000-8000-00805F9B34FB"
    }

    init {
        if (bluetoothAdapter == null) {
            Log.d(BLUE, "Null adapters")
        }
    }
}
