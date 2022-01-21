package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.o3dr.services.android.lib.drone.mission.item.command.RawMissionCommand
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class RawMissionCommandImpl : MissionCMD {
    var command = 0
    var param1 = 0f
    var param2 = 0f
    var param3 = 0f
    var param4 = 0f
    var x = 0f
    var y = 0f
    var z = 0f
    var target_system: Short = 0
    var target_component: Short = 0

    constructor(input: MissionItemImpl?) : super(input) {}
    constructor(mission: Mission?) : super(mission) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    fun setTo(s: RawMissionCommand): RawMissionCommandImpl {
        command = s.command
        param1 = s.param1
        param2 = s.param2
        param3 = s.param3
        param4 = s.param4
        x = s.x
        y = s.y
        z = s.z
        target_system = s.target_system
        target_component = s.target_component
        return this
    }

    override fun unpackMAVMessage(msg: msg_mission_item) {
        param1 = msg.param1
        param2 = msg.param2
        param3 = msg.param3
        param4 = msg.param4
        x = msg.x
        y = msg.y
        z = msg.z
        command = msg.command
        target_system = msg.target_system
        target_component = msg.target_component
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val msg = list[0]
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.x = x
        msg.y = y
        msg.z = z
        msg.command = command
        msg.target_component = target_component
        msg.target_system = target_system
        return list
    }

    override fun getType(): MissionItemType = MissionItemType.RAW_COMMAND
}
