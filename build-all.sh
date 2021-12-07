#!/bin/sh

echo "Building DroneKit"

./gradlew Mavlink:jar && ./gradlew ClientLib:assembleRelease

