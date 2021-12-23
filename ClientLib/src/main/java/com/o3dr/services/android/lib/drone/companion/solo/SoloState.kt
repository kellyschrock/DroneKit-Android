package com.o3dr.services.android.lib.drone.companion.solo

import com.o3dr.services.android.lib.drone.property.DroneAttribute
import android.util.SparseArray
import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSetting
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerMode.ControllerMode
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.android.client.utils.TxPowerComplianceCountries
import android.os.Parcel
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVMessageParser
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.companion.solo.SoloState
import java.nio.ByteBuffer

/**
 * Stores state information for the sololink companion computer.
 * Created by Fredia Huya-Kouadio on 7/10/15.
 */
class SoloState : DroneAttribute {
    var wifiSsid: String? = null
        private set
    var wifiPassword: String? = null
        private set
    var controllerVersion: String? = null
        private set
    var controllerFirmwareVersion: String? = null
        private set
    var vehicleVersion: String? = null
        private set
    var autopilotVersion: String? = null
        private set
    var gimbalVersion: String? = null
        private set
    var txPowerCompliantCountry: String? = null
        private set
    private var buttonSettings: SparseArray<SoloButtonSetting>? = null

    @get:ControllerMode
    @ControllerMode
    var controllerMode = 0
        private set

    @get:ControllerUnit
    @ControllerUnit
    var controllerUnit: String? = null
        private set

    constructor() {}
    constructor(autopilotVersion: String?, controllerFirmwareVersion: String?,
                controllerVersion: String?, vehicleVersion: String?,
                wifiPassword: String?, wifiSsid: String?, txPowerCompliantCountry: String?,
                buttonSettings: SparseArray<SoloButtonSetting>?, gimbalVersion: String?,
                @ControllerMode controllerMode: Int, @ControllerUnit controllerUnit: String?) {
        this.autopilotVersion = autopilotVersion
        this.controllerFirmwareVersion = controllerFirmwareVersion
        this.controllerVersion = controllerVersion
        this.vehicleVersion = vehicleVersion
        this.wifiPassword = wifiPassword
        this.wifiSsid = wifiSsid
        this.txPowerCompliantCountry = txPowerCompliantCountry
        this.buttonSettings = buttonSettings
        this.gimbalVersion = gimbalVersion
        this.controllerMode = controllerMode
        this.controllerUnit = controllerUnit
    }

    private val isEUTxPowerCompliant: Boolean
        private get() = TxPowerComplianceCountries.defaultCountry.name != txPowerCompliantCountry

    fun getButtonSetting(buttonType: Int): SoloButtonSetting {
        return buttonSettings!![buttonType]
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(wifiSsid)
        dest.writeString(wifiPassword)
        dest.writeString(controllerVersion)
        dest.writeString(controllerFirmwareVersion)
        dest.writeString(vehicleVersion)
        dest.writeString(autopilotVersion)
        dest.writeByte(if (isEUTxPowerCompliant) 1.toByte() else 0.toByte())
        val buttonCount = buttonSettings!!.size()
        dest.writeInt(buttonCount)

        for (i in 0 until buttonCount) {
            val buttonSetting = buttonSettings!!.valueAt(i)
            if (buttonSetting == null) {
                dest.writeInt(0)
                continue
            }
            val buttonData = buttonSetting.toBytes()
            dest.writeInt(buttonData.size)
            dest.writeByteArray(buttonData)
        }

        dest.writeString(gimbalVersion)
        dest.writeInt(controllerMode)
        dest.writeString(controllerUnit)
        dest.writeString(txPowerCompliantCountry)
    }

    protected constructor(`in`: Parcel) {
        wifiSsid = `in`.readString()
        wifiPassword = `in`.readString()
        controllerVersion = `in`.readString()
        controllerFirmwareVersion = `in`.readString()
        vehicleVersion = `in`.readString()
        autopilotVersion = `in`.readString()
        //Throw away byte that was added to ensure backwards compatibility
        `in`.readByte()
        val buttonCount = `in`.readInt()
        buttonSettings = SparseArray(buttonCount)

        for (i in 0 until buttonCount) {
            val dataSize = `in`.readInt()
            if (dataSize == 0) continue
            val dataBuffer = ByteBuffer.allocate(dataSize)
            `in`.readByteArray(dataBuffer.array())
            val buttonsList = TLVMessageParser.parseTLVPacket(dataBuffer)
            if (!buttonsList.isEmpty()) {
                for (tlvPacket in buttonsList) {
                    if (tlvPacket is SoloButtonSetting) {
                        val button = tlvPacket
                        buttonSettings!!.put(button.button, button)
                    }
                }
            }
        }

        gimbalVersion = `in`.readString()
        @ControllerMode val tempMode = `in`.readInt()
        controllerMode = tempMode
        @ControllerUnit val tempUnit = `in`.readString()
        controllerUnit = tempUnit
        txPowerCompliantCountry = `in`.readString()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<SoloState?> = object : Parcelable.Creator<SoloState?> {
            override fun createFromParcel(source: Parcel): SoloState? {
                return SoloState(source)
            }

            override fun newArray(size: Int): Array<SoloState?> {
                return arrayOfNulls(size)
            }
        }
    }
}
