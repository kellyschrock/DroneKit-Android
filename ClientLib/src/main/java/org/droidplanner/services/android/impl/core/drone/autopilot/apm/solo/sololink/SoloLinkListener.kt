package org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.sololink

import com.o3dr.services.android.lib.drone.companion.solo.tlv.SoloButtonSetting
import com.o3dr.services.android.lib.drone.companion.solo.tlv.TLVPacket
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.solo.AbstractLinkManager

/** Created by Fredia Huya-Kouadio on 7/10/15. */
interface SoloLinkListener : AbstractLinkManager.LinkListener {
    fun onTlvPacketReceived(packet: TLVPacket?)
    fun onPresetButtonLoaded(buttonType: Int, buttonSettings: SoloButtonSetting?)
}
