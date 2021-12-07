@echo off

echo Building Dronekit

.\gradlew Mavlink:jar
.\gradlew ClientLib:assembleRelease
