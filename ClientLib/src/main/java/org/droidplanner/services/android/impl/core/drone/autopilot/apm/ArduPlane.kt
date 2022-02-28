package org.droidplanner.services.android.impl.core.drone.autopilot.apm

import android.content.Context
import android.os.Handler
import com.MAVLink.common.msg_global_position_int
import com.MAVLink.common.msg_vfr_hud
import com.MAVLink.enums.MAV_TYPE
import org.droidplanner.services.android.impl.communication.service.MAVLinkClient
import org.droidplanner.services.android.impl.core.drone.LogMessageListener
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser

private val TAG = ArduPlane::class.java.simpleName

class ArduPlane(
        droneId: String?,
        context: Context?,
        mavClient: MAVLinkClient,
        handler: Handler,
        warningParser: AutopilotWarningParser?,
        logListener: LogMessageListener?)
: ArduPilot(droneId, context, mavClient, handler, warningParser, logListener) {
    override val type: Int
        get() = MAV_TYPE.MAV_TYPE_FIXED_WING

    override fun setType(type: Int) {}

    override val firmwareType: FirmwareType
        get() = FirmwareType.ARDU_PLANE

    override fun processVfrHud(vfrHud: msg_vfr_hud?) {
        // Note: Don't use vfrHud altitude, it seems grossly inaccurate.
        setGroundAndAirSpeeds(vfrHud!!.groundspeed.toDouble(), vfrHud.airspeed.toDouble(), vfrHud.climb.toDouble())
    }

    /**
     * Used to update the vehicle location, and altitude.
     * @param gpi
     */
    override fun processGlobalPositionInt(gpi: msg_global_position_int) {
        if (gpi == null) return
        super.processGlobalPositionInt(gpi)
        val relativeAlt = gpi.relative_alt / 1000.0

//        final double groundSpeedX = gpi.vx / 100.0;
//        final double groundSpeedY = gpi.vy / 100.0;
//        final double groundSpeed = Math.sqrt(Math.pow(groundSpeedX, 2) + Math.pow(groundSpeedY, 2));
//
//        final double climbRate = gpi.vz / 100.0;
        setAltitudes(relativeAlt)
    }
}
