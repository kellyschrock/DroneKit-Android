package org.droidplanner.services.android.impl.core.drone.autopilot.apm

import android.content.Context
import android.os.Handler
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.enums.MAV_TYPE
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkProvider
import org.droidplanner.services.android.impl.core.drone.LogMessageListener
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser

class ArduRover(droneId: String?, context: Context?, mavClient: DataLinkProvider<MAVLinkMessage?>?, handler: Handler?, warningParser: AutopilotWarningParser?, logListener: LogMessageListener?) : ArduPilot(droneId, context, mavClient, handler, warningParser, logListener) {
    override val type: Int
        get() = MAV_TYPE.MAV_TYPE_GROUND_ROVER

    public override fun setType(type: Int) {}
    override val firmwareType: FirmwareType
        get() = FirmwareType.ARDU_ROVER
}
