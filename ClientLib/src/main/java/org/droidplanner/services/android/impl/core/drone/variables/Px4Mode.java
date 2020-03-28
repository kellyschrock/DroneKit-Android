package org.droidplanner.services.android.impl.core.drone.variables;

import android.util.Log;

import com.MAVLink.enums.MAV_TYPE;
import com.o3dr.services.android.lib.drone.property.VehicleMode;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

public enum Px4Mode implements BaseMode<Px4Mode> {
    MANUAL("Manual",
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_MANUAL,
            0, true, true, true),
    STABILIZED("Stabilized",
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_STABILIZED,
            0, true, true, true),
    ACRO("Acro",
            Px4Util.CUSTOM_ENABLED | Px4Util.RAW_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_ACRO,
            0, true, true, true),
    RATTITUDE("Rattitude",
            Px4Util.CUSTOM_ENABLED | Px4Util.RAW_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_RATTITUDE,
            0, true, true, true),
    ALTCTL("ALTCTL",
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_ALTCTL,
            0, true, true, true),
    POSCTL("POSCTL",
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_POSCTL,
            0, true, true, true),
    LOITER("Loiter",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_LOITER, true, true, true),
    MISSION("Mission",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_MISSION, true, true, true),
    RTL("RTL",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_RTL, true, true, true),
    FOLLOW_ME("Follow Me",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET, true, false, true),
    OFFBOARD("Offboard",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, true, false, true),
    LAND("Land",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_LAND, false, true, true),
    READY("Ready",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_READY, false, true, true),
    RTGS("RTGS",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_RTGS, false, true, true),
    TAKEOFF("Takeoff",
            Px4Util.CUSTOM_ENABLED | Px4Util.AUTO_MODE_FLAGS,
            Px4Util.PX4_CUSTOM_MAIN_MODE_AUTO,
            Px4Util.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF, false, true, true),
    UNKNOWN("Unknown",
            Px4Util.CUSTOM_ENABLED | Px4Util.MANUAL_MODE_FLAGS,
            0, 0, false, false, false)
    ;

    private static final String TAG = Px4Mode.class.getSimpleName();

    private final int mainMode;
    private final int customMode;
    private final int customSubMode;
    private final boolean canBeSet;
    private final boolean fixedWingCompat;
    private final boolean multiRotorCompat;

    private final String name;

    Px4Mode(String name, int mainMode, int customMode, int customSubMode, boolean canSet, boolean fwCompat, boolean mcCompat) {
        this.name = name;
        this.mainMode = mainMode;
        this.customMode = customMode;
        this.customSubMode = customSubMode;
        this.canBeSet = canSet;
        this.fixedWingCompat = fwCompat;
        this.multiRotorCompat = mcCompat;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Px4Mode getNativeMode() {
        return this;
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

    public boolean canBeSet() { return canBeSet; }
    public boolean isFixedWing() { return fixedWingCompat; }
    public boolean isMultiRotor() { return multiRotorCompat; }

    public long getModeChangeValue() {
        final ByteBuffer buf = ByteBuffer.allocate(8);
        buf.order(ByteOrder.BIG_ENDIAN);
        buf.putInt(0);
        buf.putShort((short)customMode);
        buf.putShort((short)customSubMode);
        buf.rewind();
        return buf.getLong();
    }

    public static List<VehicleMode> getUserModesForType(int type) {
        final List<VehicleMode> modes = new ArrayList<>();

        for(Px4Mode p: userModesForType(type)) {
            final VehicleMode vm = toVehicleMode(p, type);
            if(vm != null && vm != VehicleMode.UNKNOWN) {
                modes.add(vm);
            } else {
//                Log.v(TAG, String.format("No mode found for %s", p));
            }
        }

        return modes;
    }

    public static Px4Mode[] userModesForType(int type) {
        final List<Px4Mode> list = new ArrayList<>();

        if(isCopter(type)) {
            for(Px4Mode m: values()) {
                if(m.canBeSet && m.multiRotorCompat) {
                    list.add(m);
                }
            }
        } else if(isPlane(type)) {
            for(Px4Mode m: values()) {
                if(m.canBeSet && m.fixedWingCompat) {
                    list.add(m);
                }
            }
        } else {
            // TODO: Need rover types
            list.add(Px4Mode.UNKNOWN);
        }

        return list.toArray(new Px4Mode[list.size()]);
    }

    public static Px4Mode getPx4Mode(VehicleMode mode, int type) {
        if(isCopter(type)) {
            type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        }

        return Px4Util.toPx4Mode(mode, type);
    }

    public static VehicleMode toVehicleMode(Px4Mode mode, int type) {
        if(isCopter(type)) {
            return Px4Util.toCopterMode(mode);
        } else if(isPlane(type)) {
            return Px4Util.toPlaneMode(mode);
        } else {
            return VehicleMode.UNKNOWN;
        }
    }

    public static Px4Mode getHeartbeatMode(int baseMode, long customMode, int type) {
        for(Px4Mode mode: values()) {
            if(mode.mainMode == baseMode) {
                if(mode.customMode == customMode) {
                    return mode;
                }
            }
        }

        return Px4Mode.UNKNOWN;
    }

    public static boolean isValid(Px4Mode mode) {
        return (mode != null && mode != UNKNOWN);
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

    public static boolean isPlane(int type) {
        switch(type) {
            case MAV_TYPE.MAV_TYPE_FIXED_WING:
            case MAV_TYPE.MAV_TYPE_FLAPPING_WING:
                return true;

            default:
                return false;
        }
    }
}
