package org.droidplanner.services.android.impl.core.drone.variables;

import com.MAVLink.enums.MAV_TYPE;
import com.o3dr.services.android.lib.drone.property.VehicleMode;

public enum Px4Modes {
//	FIXED_WING_MANUAL (0,"Manual",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_CIRCLE (1,"Circle",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_STABILIZE (2,"Stabilize",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_TRAINING (3,"Training",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_ACRO(4, "Acro", MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_FLY_BY_WIRE_A (5,"FBW A",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_FLY_BY_WIRE_B (6,"FBW B",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_CRUISE(7, "Cruise", MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_AUTOTUNE(8, "AutoTune", MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_AUTO (10,"Auto",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_RTL (11,"RTL",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_LOITER (12,"Loiter",MAV_TYPE.MAV_TYPE_FIXED_WING),
//	FIXED_WING_GUIDED (15,"Guided",MAV_TYPE.MAV_TYPE_FIXED_WING),
//
//	ROTOR_STABILIZE(0, "Stabilize", MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_ACRO(1,"Acro", MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_ALT_HOLD(2, "Alt Hold",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_AUTO(3, "Auto",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_GUIDED(4, "Guided",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_LOITER(5, "Loiter",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_RTL(6, "RTL",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_CIRCLE(7, "Circle",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_LAND(9, "Land",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_TOY(11, "Drift",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_SPORT(13, "Sport",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_FLIP(14, "Flip",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_AUTOTUNE(15, "Autotune",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_POSHOLD(16, "PosHold",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_BRAKE(17,"Brake",MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_THROW(18, "Throw", MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_AVOID_ADSB(19,"Avoid ADSB", MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_GUIDED_NOGPS(20,"Guided NoGPS", MAV_TYPE.MAV_TYPE_QUADROTOR),
//	ROTOR_SMART_RTL(21, "Smart RTL", MAV_TYPE.MAV_TYPE_QUADROTOR),
//
//	ROVER_MANUAL(0, "MANUAL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_ACRO(1, "ACRO", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_LEARNING(2, "LEARNING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_STEERING(3, "STEERING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_HOLD(4, "HOLD", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_AUTO(10, "AUTO", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_RTL(11, "RTL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_SMARTRTL(12, "SMART_RTL", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_GUIDED(15, "GUIDED", MAV_TYPE.MAV_TYPE_GROUND_ROVER),
//	ROVER_INITIALIZING(16, "INITIALIZING", MAV_TYPE.MAV_TYPE_GROUND_ROVER),

    MANUAL("Manual", Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_MANUAL, 0),
    STABILIZED("Stabilized", Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_STABILIZED, 0),
    ACRO("Acro", Px4Util.CUSTOM_ENABLED | Px4Util.RAW_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_ACRO, 0),
    RATTITUDE("Rattitude", Px4Util.CUSTOM_ENABLED | Px4Util.RAW_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_RATTITUDE, 0),
    ALTCTL("ALTCTL", Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_ALTCTL, 0),
    POSCTL("POSCTL", Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_POSCTL, 0),
    LOITER("Loiter", Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO, Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    MISSION("Mission", Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO, Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_MISSION),
    RTL("RTL", Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO, Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_RTL),
    FOLLOW_ME("Follow Me", Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO, Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET),
    OFFBOARD("Offboard", Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS, Px4Util.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0),
    UNKNOWN("Unknown", Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS, 0, 0)
    ;

    private final int mainMode;
    private final int customMode;
    private final int customSubMode;

    private final String name;

    Px4Modes(String name, int mainMode, int customMode, int customSubMode) {
        this.name = name;
        this.mainMode = mainMode;
        this.customMode = customMode;
        this.customSubMode = customSubMode;
    }

//	Px4Modes(long number, String name, int type){
//		this.number = number;
//		this.name = name;
//		this.type = type;
//	}

    public String getName() {
        return name;
    }

    public int getMainMode() {
        return mainMode;
    }

    public int getCustomMode() {
        return customMode;
    }

    public int getCustomSubMode() {
        return customSubMode;
    }

    public static Px4Modes getMode(VehicleMode mode, int type) {
        if(isCopter(type)) {
            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        }

        return Px4Util.toPx4Mode(mode, type);
    }

//    public static Px4Modes getMode(long i, int type) {
//        if (isCopter(type)) {
//            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
//        }
//
//        for (Px4Modes mode : Px4Modes.values()) {
//            if (i == mode.getNumber() && type == mode.getType()) {
//                return mode;
//            }
//        }
//
//        return UNKNOWN;
//    }

//    public static Px4Modes getMode(String str, int type) {
//        if (isCopter(type)) {
//            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
//        }
//
//        for (Px4Modes mode : Px4Modes.values()) {
//            if (str.equals(mode.getName()) && type == mode.getType()) {
//                return mode;
//            }
//        }
//        return UNKNOWN;
//    }

//    public static List<Px4Modes> getModeList(int type) {
//        List<Px4Modes> modeList = new ArrayList<Px4Modes>();
//
//        if (isCopter(type)) {
//            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
//        }
//
//        for (Px4Modes mode : Px4Modes.values()) {
//            if (mode.getType() == type) {
//                modeList.add(mode);
//            }
//        }
//        return modeList;
//    }

    public static boolean isValid(Px4Modes mode) {
        return mode != Px4Modes.UNKNOWN;
    }

    public static boolean isCopter(int type) {
        switch (type) {
            case MAV_TYPE.MAV_TYPE_TRICOPTER:
            case MAV_TYPE.MAV_TYPE_QUADROTOR:
            case MAV_TYPE.MAV_TYPE_HEXAROTOR:
            case MAV_TYPE.MAV_TYPE_OCTOROTOR:
            case MAV_TYPE.MAV_TYPE_HELICOPTER:
                return true;

            default:
                return false;
        }
    }
}
