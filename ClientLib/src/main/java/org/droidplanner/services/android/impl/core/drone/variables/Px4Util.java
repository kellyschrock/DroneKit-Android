package org.droidplanner.services.android.impl.core.drone.variables;

import com.MAVLink.enums.MAV_MODE_FLAG;
import com.MAVLink.enums.MAV_TYPE;
import com.o3dr.services.android.lib.drone.property.VehicleMode;

import java.util.HashMap;

public class Px4Util {
    public static final int AUTO_MODE_FLAGS =
            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;

    public static final int CUSTOM_ENABLED = MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    public static final int RAW_MODE_FLAGS =
            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    public static final int MANUAL_MODE_FLAGS =
            RAW_MODE_FLAGS | MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED;

    public static final int PX4_CUSTOM_MAIN_MODE_MANUAL            = 1;
    public static final int PX4_CUSTOM_MAIN_MODE_ALTCTL            = 2;
    public static final int PX4_CUSTOM_MAIN_MODE_POSCTL            = 3;
    public static final int PX4_CUSTOM_MAIN_MODE_AUTO              = 4;
    public static final int PX4_CUSTOM_MAIN_MODE_ACRO              = 5;
    public static final int PX4_CUSTOM_MAIN_MODE_OFFBOARD          = 6;
    public static final int PX4_CUSTOM_MAIN_MODE_STABILIZED        = 7;
    public static final int PX4_CUSTOM_MAIN_MODE_RATTITUDE         = 8;

    public static final int PX4_CUSTOM_SUB_MODE_AUTO_READY         = 1;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF       = 2;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_LOITER        = 3;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_MISSION       = 4;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_RTL           = 5;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_LAND          = 6;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_RTGS          = 7;
    public static final int PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8;

    private static final HashMap<VehicleMode, Px4Modes> sModeMap = new HashMap<>();

    static {
        // Copter
        sModeMap.put(VehicleMode.COPTER_STABILIZE, Px4Modes.STABILIZED);
        sModeMap.put(VehicleMode.COPTER_ACRO, Px4Modes.ACRO);
        sModeMap.put(VehicleMode.COPTER_ALT_HOLD, Px4Modes.ALTCTL);
        sModeMap.put(VehicleMode.COPTER_AUTO, Px4Modes.MISSION);
        sModeMap.put(VehicleMode.COPTER_LOITER, Px4Modes.POSCTL);
        sModeMap.put(VehicleMode.COPTER_RTL, Px4Modes.RTL);
        sModeMap.put(VehicleMode.COPTER_SMART_RTL, Px4Modes.RTL);
        sModeMap.put(VehicleMode.COPTER_CIRCLE, Px4Modes.POSCTL);
        sModeMap.put(VehicleMode.COPTER_GUIDED, Px4Modes.OFFBOARD);

        // Plane
        sModeMap.put(VehicleMode.PLANE_MANUAL, Px4Modes.MANUAL);
        sModeMap.put(VehicleMode.PLANE_ACRO, Px4Modes.ACRO);
        sModeMap.put(VehicleMode.PLANE_STABILIZE, Px4Modes.STABILIZED);
        sModeMap.put(VehicleMode.PLANE_TRAINING, Px4Modes.STABILIZED);
        sModeMap.put(VehicleMode.PLANE_FLY_BY_WIRE_A, Px4Modes.ALTCTL);
        sModeMap.put(VehicleMode.PLANE_FLY_BY_WIRE_B, Px4Modes.POSCTL);
        sModeMap.put(VehicleMode.PLANE_CRUISE, Px4Modes.POSCTL);
        sModeMap.put(VehicleMode.PLANE_AUTO, Px4Modes.MISSION);
        sModeMap.put(VehicleMode.PLANE_RTL, Px4Modes.RTL);
        sModeMap.put(VehicleMode.PLANE_LOITER, Px4Modes.LOITER);
        sModeMap.put(VehicleMode.PLANE_GUIDED, Px4Modes.OFFBOARD);

        // Rover
        sModeMap.put(VehicleMode.ROVER_MANUAL, Px4Modes.MANUAL);
        sModeMap.put(VehicleMode.ROVER_ACRO, Px4Modes.ACRO);
        sModeMap.put(VehicleMode.ROVER_LEARNING, Px4Modes.STABILIZED);
        sModeMap.put(VehicleMode.ROVER_STEERING, Px4Modes.STABILIZED);
        sModeMap.put(VehicleMode.ROVER_AUTO, Px4Modes.MISSION);
        sModeMap.put(VehicleMode.ROVER_RTL, Px4Modes.RTL);
        sModeMap.put(VehicleMode.ROVER_SMART_RTL, Px4Modes.RTL);
        sModeMap.put(VehicleMode.ROVER_GUIDED, Px4Modes.OFFBOARD);
    }

    static Px4Modes toPx4Mode(VehicleMode mode, int mavType) {
        Px4Modes px4Mode = sModeMap.get(mode);
        if(px4Mode == null) {
            // Unsupported mode
            switch(mavType) {
                case MAV_TYPE.MAV_TYPE_QUADROTOR: {
                    px4Mode = Px4Modes.POSCTL;
                    break;
                }

                case MAV_TYPE.MAV_TYPE_FIXED_WING:
                case MAV_TYPE.MAV_TYPE_GROUND_ROVER:
                case MAV_TYPE.MAV_TYPE_SURFACE_BOAT:
                default: {
                    px4Mode = Px4Modes.STABILIZED;
                    break;
                }
            }
        }

        return px4Mode;
    }
}
