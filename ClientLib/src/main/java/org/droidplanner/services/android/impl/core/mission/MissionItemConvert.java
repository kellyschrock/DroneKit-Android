package org.droidplanner.services.android.impl.core.mission;

import com.MAVLink.common.msg_mission_item;
import com.MAVLink.common.msg_mission_item_int;
import java.util.ArrayList;
import java.util.List;

public class MissionItemConvert {
    public static msg_mission_item toMissionItem(msg_mission_item_int input) {
        final msg_mission_item item = new msg_mission_item();

        item.mission_type = input.mission_type;
        item.command = input.command;
        item.target_component = input.target_component;
        item.target_system = input.target_system;
        item.autocontinue = input.autocontinue;
        item.current = input.current;
        item.frame = input.frame;
        item.param1 = input.param1;
        item.param2 = input.param2;
        item.param3 = input.param3;
        item.param4 = input.param4;
        item.x = (float)(input.x / 1e7);
        item.y = (float)(input.y / 1e7);
        item.z = input.z;
        item.seq = input.seq;
        return item;
    }

    public static List<msg_mission_item> toMissionItems(Iterable<msg_mission_item_int> input) {
        final List<msg_mission_item> output = new ArrayList<>();

        for(msg_mission_item_int item: input) {
            output.add(toMissionItem(item));
        }

        return output;
    }

    public static List<msg_mission_item_int> toMissionItemInts(Iterable<msg_mission_item> input) {
        final List<msg_mission_item_int> output = new ArrayList<>();

        for(msg_mission_item item: input) {
            output.add(toMissionItemInt(item));
        }

        return output;
    }

    public static msg_mission_item_int toMissionItemInt(msg_mission_item input) {
        final msg_mission_item_int o = new msg_mission_item_int();

        o.autocontinue = input.autocontinue;
        o.command = input.command;
        o.mission_type = input.mission_type;
        o.seq = input.seq;
        o.current = input.current;
        o.frame = input.frame;
        o.param1 = input.param1;
        o.param2 = input.param2;
        o.param3 = input.param3;
        o.param4 = input.param4;
        o.x = (int)(input.x * 1e7);
        o.y = (int)(input.y * 1e7);
        o.z = input.z;
        o.target_component = input.target_component;
        o.target_system = input.target_system;

        return o;
    }
}
