package com.o3dr.services.android.lib.drone.attribute.error

/**
 * List the possible command execution errors.
 * Created by Fredia Huya-Kouadio on 6/24/15.
 */
object CommandExecutionError {
    /**
     * Command execution was temporarily rejected. You may try again at a later time.
     */
    const val COMMAND_TEMPORARILY_REJECTED = 1

    /**
     * Command execution was denied.
     */
    const val COMMAND_DENIED = 2

    /**
     * Command is not supported by the target autopilot.
     */
    const val COMMAND_UNSUPPORTED = 3

    /**
     * Command execution failed.
     */
    const val COMMAND_FAILED = 4
}
