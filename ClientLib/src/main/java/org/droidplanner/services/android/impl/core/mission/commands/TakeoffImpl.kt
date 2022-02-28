package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.commands.MissionCMD
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class TakeoffImpl : MissionCMD {
    var finishedAlt = 10.0
    var pitch = 0.0

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, altitude: Double) : super(mission) {
        finishedAlt = altitude
        pitch = 0.0
    }

    constructor(mission: Mission?, altitude: Double, pitch: Double) : super(mission) {
        finishedAlt = altitude
        this.pitch = pitch
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_TAKEOFF
        mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
        mavMsg.z = finishedAlt.toFloat()
        if (pitch > 0) mavMsg.param1 = pitch.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        finishedAlt = mavMsg.z.toDouble()
        pitch = mavMsg.param1.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.TAKEOFF

    companion object {
        const val DEFAULT_TAKEOFF_ALTITUDE = 10.0
    }
}
