package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class LoiterToAltImpl : MissionCMD {
    var lat = 0.0
    var lng = 0.0
    var alt = 0.0

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, lat: Double, lng: Double, alt: Double) : super(mission) {
        this.lat = lat
        this.lng = lng
        this.alt = alt
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT
        mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
        mavMsg.param1 = 0f // No heading required
        mavMsg.param2 = 0f // 0 radius (use standard loiter radius)
        mavMsg.param4 = 0f // Center wp of loiter
        mavMsg.x = lat.toFloat()
        mavMsg.y = lng.toFloat()
        mavMsg.z = alt.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        lat = mavMsg.x.toDouble()
        lng = mavMsg.y.toDouble()
        alt = mavMsg.z.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.LOITER_TO_ALT
}
