# Swerve Debug Documentation

Swerve debug is designed to make alignment of the swerve drive easier. All information is provided in the `Swerve Debug` tab, and comes from the direction each swerve module is facing. This information can be activated by running the `Test` mode in the Driver Station.

## What each section does

The menu is divided into four sections, `Angle`, `Raw Angle`, `Offset`, and `Test Offset`.

### Angle

`Angle` gives the output after accounting for the offset.

### Raw Angle

`Raw Angle` gives the direct output from the wheels.

### Offset

`Offset` is an angle value which is applied to `Raw Angle` to get `Angle`.

### Test Offset

`Test Offset` measures how far each wheel is turned from their position when the robot is enabled.

1. Enable the robot in `Test` mode.
2. Rotate the wheel(s) until they are in the correct position(s).
3. Copy the output(s) from `Test Offset` and paste it/them into the file coresponding to the robot configuration file being worked on (located in `src\main\deploy`), overwriting the previous value(s) on lines 82 to 85.
