package com.something;

import junit.framework.TestCase;

import org.droidplanner.services.android.impl.core.drone.variables.Px4Mode;
import org.droidplanner.services.android.impl.core.drone.variables.Px4Util;
import org.junit.Test;

public class TestPx4Mode {

    @Test
    public void testReadModes() {
        for(Px4Mode mode: Px4Mode.values()) {
            final long mcv = mode.getModeChangeValue();
            final Px4Util.Px4CustomMode customMode = new Px4Util.Px4CustomMode(mcv);
            say(String.format("%s: mode.mainMode=%d .customMode=%d .customSubMode=%d", mode.name(), mode.getMainMode(), mode.getCustomMode(), mode.getCustomSubMode()));
            say(String.format("%s: customMode.main_mode=%d .sub_mode=%d", mode.name(), customMode.main_mode, customMode.sub_mode));
            TestCase.assertEquals(mode.getCustomMode(), customMode.sub_mode);
        }
    }

    @Test
    public void testWritePx4Modes() {
        for(Px4Mode mode: Px4Mode.values()) {
            final long custom_mode = mode.getModeChangeValue();
            say(String.format("custom_mode for %s is %d", mode.name(), custom_mode));
            final Px4Util.Px4CustomMode customMode = new Px4Util.Px4CustomMode(custom_mode);
            TestCase.assertEquals(mode.getCustomMode(), customMode.sub_mode);
        }
    }

    private void say(String str) {
        System.out.println(str);
    }
}
