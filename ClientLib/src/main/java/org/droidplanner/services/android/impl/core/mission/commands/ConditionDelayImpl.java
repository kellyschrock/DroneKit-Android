package org.droidplanner.services.android.impl.core.mission.commands;

import com.MAVLink.common.msg_mission_item;
import com.MAVLink.enums.MAV_CMD;
import com.MAVLink.enums.MAV_FRAME;

import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools;
import org.droidplanner.services.android.impl.core.mission.Mission;
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl;
import org.droidplanner.services.android.impl.core.mission.MissionItemType;

import java.util.List;

public class ConditionDelayImpl extends MissionCMD {
	private double seconds = 0;

	public ConditionDelayImpl(MissionItemImpl item) {
		super(item);
	}

	public ConditionDelayImpl(msg_mission_item msg, Mission mission) {
		super(mission);
		unpackMAVMessage(msg);
	}

	public ConditionDelayImpl(Mission mission, double seconds) {
		super(mission);
		setSeconds(seconds);
	}
	
	@Override
	public List<msg_mission_item> packMissionItem() {
		List<msg_mission_item> list = super.packMissionItem();
		msg_mission_item mavMsg = list.get(0);
		mavMsg.command = MAV_CMD.MAV_CMD_CONDITION_DELAY;
		mavMsg.frame = MAV_FRAME.MAV_FRAME_MISSION;
		mavMsg.param1 = (float)seconds;
		return list;
	}

	@Override
	public void unpackMAVMessage(msg_mission_item mavMsg) {
		seconds = mavMsg.param1;
	}

	@Override
	public MissionItemType getType() {
		return MissionItemType.CONDITION_DELAY;
	}

	public double getSeconds() { return seconds; }
	public void setSeconds(double seconds) { this.seconds = seconds; }
}
