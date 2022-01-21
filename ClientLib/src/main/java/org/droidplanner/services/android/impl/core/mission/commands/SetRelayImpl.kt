package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.commands.MissionCMD
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

/**
 * Mavlink message builder for the 'SetRelay' mission item.
 * Set a Relay pin’s voltage high or low.
 */
class SetRelayImpl : MissionCMD {
    var relayNumber = 0
        private set
    var isEnabled = false
        private set

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, relayNumber: Int, enabled: Boolean) : super(mission) {
        this.relayNumber = relayNumber
        isEnabled = enabled
    }

    override fun getType(): MissionItemType = MissionItemType.SET_RELAY

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        relayNumber = mavMsg.param1.toInt()
        isEnabled = mavMsg.param2 != 0f
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_SET_RELAY
        mavMsg.param1 = relayNumber.toFloat()
        mavMsg.param2 = if (isEnabled) 1.toFloat() else 0.toFloat()
        return list
    }
}
