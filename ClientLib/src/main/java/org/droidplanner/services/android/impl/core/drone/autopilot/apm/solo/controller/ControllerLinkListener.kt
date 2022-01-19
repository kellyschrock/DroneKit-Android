package org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.controller

import com.o3dr.services.android.lib.drone.companion.solo.button.ButtonPacket
import com.o3dr.services.android.lib.drone.companion.solo.controller.SoloControllerUnits.ControllerUnit
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.AbstractLinkManager

interface ControllerLinkListener : AbstractLinkManager.LinkListener {
    fun onTlvPacketReceived(packet: TLVPacket?)
    fun onWifiInfoUpdated(wifiName: String, wifiPassword: String)
    fun onButtonPacketReceived(packet: ButtonPacket)
    fun onTxPowerComplianceCountryUpdated(compliantCountry: String)
    fun onControllerModeUpdated()
    fun onControllerUnitUpdated(@ControllerUnit trimmedResponse: String)
}
