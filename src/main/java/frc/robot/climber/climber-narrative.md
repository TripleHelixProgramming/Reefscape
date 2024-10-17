# Unit conversions

## Position

The position of the motor's internal encoder is read in native units of [[rotations of motor]](https://codedocs.revrobotics.com/java/com/revrobotics/sparkrelativeencoder#getPosition()).
>```java
>public double getPosition()
>```
>Get the position of the motor. This returns the native units of 'rotations' by default, and can be changed by a scale factor using `setPositionConversionFactor()`.

The parameter we pass to [`setPositionConversionFactor()`](https://codedocs.revrobotics.com/java/com/revrobotics/sparkrelativeencoder#setPositionConversionFactor(double)) will need to have units of [desired unit]/[native unit].

Our desired unit for controlling the position of the climber actuator is [in of actuator travel]. So we need to calculate the conversion factor [in of actuator travel]/[rotations of motor].

Knowns:
- Pulley reduction = 36 [rotations of motor] / 24 [rotations of threaded rod]
- Pitch of threaded rod = 10 [rotation of threaded rod] / 1 [in of actuator travel]

Assembling these by unit analysis, we get:

```java
public static final double kGearRatio = 36.0 / 24.0; // pulley ratio
public static final double kPitch = 10.0; // turns per inch

public static final double kPositionConversionFactor =
  1.0 / (kGearRatio * kPitch); // inches per rotation
```

## Velocity

The velocity of the motor's internal encoder is read in native units of [[RPM of motor]](https://codedocs.revrobotics.com/java/com/revrobotics/sparkrelativeencoder#getVelocity()).
>```java
>public double getVelocity()
>```
>Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be changed by a scale factor using setVelocityConversionFactor().

The parameter we pass to [`setVelocityConversionFactor()`](https://codedocs.revrobotics.com/java/com/revrobotics/sparkrelativeencoder#setVelocityConversionFactor(double)) will need to have units of [desired unit]/[native unit].

Our desired unit for controlling the velocity of the climber actuator is [in/s of actuator travel]. So we need to calculate the conversion factor [in/s of actuator travel]/[RPM of motor].

Knowns:
- `kPositionConversionFactor` is in units of [in of actuator travel]/[rotations of motor]
- 60 [s] = 1 [m]

Assembling these by unit analysis, we get:

```java
public static final double kVelocityConversionFactor =
  kPositionConversionFactor / 60.0; // in/s per RPM
```

# Maximum speed and acceleration

## Maximum velocity

Our desired unit for controlling the velocity of the climber actuator is [in/s of actuator travel].

Knowns:
- `kVelocityConversionFactor` is in units of [in/s of actuator travel]/[RPM of motor]
- Free speed of a REV NEO motor is approximately [5880 [RPM]](https://www.reca.lc/motors)
- We would like to operate the climber actuator at a maximum speed of approximately 80% of the motor's free speed (an arbitrary choice)

Assembling these by unit analysis, we get:

```java
public static final double kRapidMaxVelocity = (0.8 * 5880.0) * kVelocityConversionFactor;
```

## Maximum acceleration

Our desired unit for controlling the acceleration of the climber actuator is [in/s^2 of actuator travel].

Knowns:
- `kRapidMaxVelocity` is in units of [in/s of actuator travel]
- We would like to accelerate to the maximum speed in approximately 0.25 [s] (an arbitrary choice)

Assembling these by unit analysis, we get:

```java
public static final double kRapidMaxAcceleration = kRapidMaxVelocity / 0.25;
```
