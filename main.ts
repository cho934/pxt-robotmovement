/**
 * Odometry module for MicroBit
 * Calculates position and orientation using wheel encoder data
 */
//% color="#0000BB" weight=99 block="Odometry"
namespace odometry {
    // Global variables for position tracking
    export let X = 0;           // X position in mm
    export let Y = 0;           // Y position in mm  
    export let alphaRad = 0;    // Orientation angle in radians

    // Configuration parameters
    export let entraxeInMM = 100;   // Distance between wheels in mm
    export let ticksPerMeter = 200000;   // Number of ticks per meter

    /**
     * Initialize the odometry module with specific parameters
     * @param trackWidth Distance between encoders' wheels in mm
     * @param ticksPerMeter Number of encoder ticks per meter
     */
    //% block="initialize odometry with trackWidth %trackWidth|mm and %ticksPerMeter|ticks per meter"
    export function initialize(entraxe_mm: number, nbticksPerMeter: number) {
        entraxeInMM = entraxe_mm;
        ticksPerMeter = nbticksPerMeter;
        X = 0;
        Y = 0;
        alphaRad = 0;
    }

    /**
     * Reset position and orientation to zero
     */
    //% block="reset odometry"
    export function reset() {
        X = 0;
        Y = 0;
        alphaRad = 0;
    }

    /**
     * Set position and orientation to specific values
     * @param x X position in mm
     * @param y Y position in mm
     * @param angle Orientation in radians
     */
    //% block="set position to x: %x|y: %y|angle: %angle"
    export function setPosition(x: number, y: number, anglerad: number) {
        X = x;
        Y = y;
        alphaRad = anglerad;
    }

    /**
     * Normalize angle to the range [-π, π]
     * @param angle Angle in radians to normalize
     * @returns Normalized angle in range [-π, π]
     */
    //% block="Normalize angle %angle to range [-π, π]"
    export function normalizeAngle(angle: number): number {
        let result = angle;
        while (result > Math.PI) {
            result -= 2 * Math.PI;
        }
        while (result <= -Math.PI) {
            result += 2 * Math.PI;
        }
        return result;
    }

    /**
     * Update odometry with new encoder values in mm
     * @param leftDeltaMm Left encoder delta in mm
     * @param rightDeltaMm Right encoder delta in mm
     */
    //% block="update with leftDelta: %leftDeltaMm|mm + rightDelta: %rightDeltaMm|mm"
    export function update(leftDeltaMm: number, rightDeltaMm: number) {
        // Calculate distance traveled and angle variation
        let deltaDist = (leftDeltaMm + rightDeltaMm) / 2;
        let diffCount = rightDeltaMm - leftDeltaMm;
        let deltaTheta = diffCount / entraxeInMM; // In radians

        if (Math.abs(diffCount) < 0.001) {
            // Movement is essentially straight
            X += deltaDist * Math.cos(alphaRad);
            Y += deltaDist * Math.sin(alphaRad);
        } else {
            // Robot follows an arc
            // Calculate the radius of curvature
            let R = deltaDist / deltaTheta;

            // Update position
            X += R * (-Math.sin(alphaRad) + Math.sin(alphaRad + deltaTheta));
            Y += R * (Math.cos(alphaRad) - Math.cos(alphaRad + deltaTheta));

            // Update heading
            alphaRad += deltaTheta;

            // Normalize angle to [-π, π]
            alphaRad = normalizeAngle(alphaRad);
        }
    }

    /**
     * Update odometry with new encoder values in ticks
     * @param leftDeltaTicks Left encoder delta in ticks
     * @param rightDeltaTicks Right encoder delta in ticks
     */
    //% block="update with leftDelta: %leftDeltaTicks|ticks + rightDelta: %rightDeltaTicks|ticks"
    export function updateFromTicks(leftDeltaTicks: number, rightDeltaTicks: number) {
        // Convert ticks to mm
        let leftDeltaMm = leftDeltaTicks * 1000 / ticksPerMeter;
        let rightDeltaMm = rightDeltaTicks * 1000 / ticksPerMeter;

        // Call the regular update function
        update(leftDeltaMm, rightDeltaMm);
    }

    /**
     * Get current X position in mm
     */
    //% block="get X position (mm)"
    export function getX(): number {
        return X;
    }

    /**
     * Get current Y position in mm
     */
    //% block="get Y position (mm)"
    export function getY(): number {
        return Y;
    }

    /**
     * Get current orientation in radians
     */
    //% block="get orientation (radians)"
    export function getOrientationRad(): number {
        return alphaRad;
    }

    /**
     * Get current orientation in degrees
     */
    //% block="get orientation (degrees)"
    export function getOrientationDegrees(): number {
        return alphaRad * 180 / Math.PI;
    }

    /**
     * Calculate distance to a point
     * @param x X coordinate of the target point in mm
     * @param y Y coordinate of the target point in mm
     */
    //% block="distance to point x: %x|y: %y"
    export function distanceTo(x: number, y: number): number {
        let dx = x - X;
        let dy = y - Y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate angle in radians to a point (relative to current orientation)
     * @param x X coordinate of the target point in mm
     * @param y Y coordinate of the target point in mm
     */
    //% block="angle to point x: %x|y: %y"
    export function angleTo(x: number, y: number): number {
        let dx = x - X;
        let dy = y - Y;
        let targetAngle = Math.atan2(dy, dx);

        // Calculate the difference and normalize to [-π, π]
        let angleDiff = targetAngle - alphaRad;
        return normalizeAngle(angleDiff);
    }
}














/**
 * MotorDrivers namespace - Collection of predefined motor drivers
 * Add your custom drivers here
 */
//% color="#FF4000" weight=98 icon="\uf1b0" block="MotorDrivers"
namespace MotorDrivers {
    // Motor direction enum
    export enum Direction {
        //% block="Forward"
        Forward = 0,
        //% block="Backward"
        Backward = 1
    }

    // Interface for all motor drivers
    export interface IMotorDriver {
        setMotorSpeed(leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction): void;
        stop(): void;
    }

    /**
     * Base class for Servo Motor Drivers
     */
    export class ServoDriver implements IMotorDriver {
        private leftPin: AnalogPin;
        private rightPin: AnalogPin;
        private leftInverted: boolean;
        private rightInverted: boolean;

        /**
         * Create a new continuous servo driver
         * @param leftPin Left servo pin
         * @param rightPin Right servo pin
         * @param leftInverted Invert left motor direction
         * @param rightInverted Invert right motor direction
         */
        constructor(leftPin: AnalogPin, rightPin: AnalogPin, leftInverted = false, rightInverted = false) {
            this.leftPin = leftPin;
            this.rightPin = rightPin;
            this.leftInverted = leftInverted;
            this.rightInverted = rightInverted;
        }

        /**
         * Set speeds for both motors
         * @param leftSpeed Left motor speed (0-100)
         * @param rightSpeed Right motor speed (0-100)
         * @param leftDir Left motor direction
         * @param rightDir Right motor direction
         */
        setMotorSpeed(leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction): void {
            // For continuous servos:
            // - 90 is stop
            // - 0-89 is one direction
            // - 91-180 is other direction

            // Constrain speeds
            leftSpeed = Math.constrain(leftSpeed, 0, 100);
            rightSpeed = Math.constrain(rightSpeed, 0, 100);

            // Calculate servo values
            let leftValue = 90;
            let rightValue = 90;

            // Apply direction and speed to left motor
            if (leftSpeed > 0) {
                let directionFactor = (leftDir === Direction.Forward) ? -1 : 1;
                if (this.leftInverted) directionFactor *= -1;
                leftValue = 90 + directionFactor * Math.map(leftSpeed, 0, 100, 0, 90);
            }

            // Apply direction and speed to right motor
            if (rightSpeed > 0) {
                let directionFactor = (rightDir === Direction.Forward) ? 1 : -1;
                if (this.rightInverted) directionFactor *= -1;
                rightValue = 90 + directionFactor * Math.map(rightSpeed, 0, 100, 0, 90);
            }

            // Ensure values are in valid range
            leftValue = Math.constrain(leftValue, 0, 180);
            rightValue = Math.constrain(rightValue, 0, 180);

            // Send commands to servos
            pins.servoWritePin(this.leftPin, leftValue);
            pins.servoWritePin(this.rightPin, rightValue);
        }

        /**
         * Stop both motors
         */
        stop(): void {
            pins.servoWritePin(this.leftPin, 90);
            pins.servoWritePin(this.rightPin, 90);
        }
    }

    /**
     * Driver for motor:bit or similar DRV8833-based motor controllers
     */
    export class MotorBitDriver implements IMotorDriver {
        private leftPin1: DigitalPin;
        private leftPin2: DigitalPin;
        private rightPin1: DigitalPin;
        private rightPin2: DigitalPin;

        /**
         * Create a new motor:bit driver
         * @param leftPin1 Left motor pin 1
         * @param leftPin2 Left motor pin 2
         * @param rightPin1 Right motor pin 1
         * @param rightPin2 Right motor pin 2
         */
        constructor(leftPin1: DigitalPin, leftPin2: DigitalPin, rightPin1: DigitalPin, rightPin2: DigitalPin) {
            this.leftPin1 = leftPin1;
            this.leftPin2 = leftPin2;
            this.rightPin1 = rightPin1;
            this.rightPin2 = rightPin2;
        }

        /**
         * Set speeds for both motors
         * @param leftSpeed Left motor speed (0-100)
         * @param rightSpeed Right motor speed (0-100)
         * @param leftDir Left motor direction
         * @param rightDir Right motor direction
         */
        setMotorSpeed(leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction): void {
            // Map 0-100 speed to 0-1023 PWM range
            let leftPwm = Math.map(leftSpeed, 0, 100, 0, 1023);
            let rightPwm = Math.map(rightSpeed, 0, 100, 0, 1023);

            // Set left motor
            if (leftDir === Direction.Forward) {
                pins.analogWritePin(this.leftPin1, leftPwm);
                pins.analogWritePin(this.leftPin2, 0);
            } else {
                pins.analogWritePin(this.leftPin1, 0);
                pins.analogWritePin(this.leftPin2, leftPwm);
            }

            // Set right motor
            if (rightDir === Direction.Forward) {
                pins.analogWritePin(this.rightPin1, rightPwm);
                pins.analogWritePin(this.rightPin2, 0);
            } else {
                pins.analogWritePin(this.rightPin1, 0);
                pins.analogWritePin(this.rightPin2, rightPwm);
            }
        }

        /**
         * Stop both motors
         */
        stop(): void {
            pins.analogWritePin(this.leftPin1, 0);
            pins.analogWritePin(this.leftPin2, 0);
            pins.analogWritePin(this.rightPin1, 0);
            pins.analogWritePin(this.rightPin2, 0);
        }
    }

    /**
     * Adapter for "motor" library
     * For compatibility with various motor modules
     */
    export class MotorLibraryAdapter implements IMotorDriver {
        private leftPin: number;
        private rightPin: number;

        /**
         * Create an adapter for the 'motor' library
         * @param leftPin Left motor pin
         * @param rightPin Right motor pin
         */
        constructor(leftPin: number, rightPin: number) {
            this.leftPin = leftPin;
            this.rightPin = rightPin;
        }

        /**
         * Set speeds for both motors
         * @param leftSpeed Left motor speed (0-100)
         * @param rightSpeed Right motor speed (0-100)
         * @param leftDir Left motor direction
         * @param rightDir Right motor direction
         */
        setMotorSpeed(leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction): void {
            // This uses an external 'motor' library
            // Adapt this implementation to match your specific library

            // Example assuming a hypothetical 'motor' namespace with motorRun method
            // Your actual implementation would reference the real library

            // Assuming motor.Dir.CW and motor.Dir.CCW constants exist
            const leftDirMap = leftDir === Direction.Forward ? 0 /* motor.Dir.CW */ : 1 /* motor.Dir.CCW */;
            const rightDirMap = rightDir === Direction.Forward ? 0 /* motor.Dir.CW */ : 1 /* motor.Dir.CCW */;

            // Uncomment and adapt to your library:
            // motor.motorRun(this.leftPin, leftDirMap, leftSpeed);
            // motor.motorRun(this.rightPin, rightDirMap, rightSpeed);

            // For now, just show a message
            serial.writeLine(`Motor L: ${leftSpeed} ${leftDir}, R: ${rightSpeed} ${rightDir}`);
        }

        /**
         * Stop both motors
         */
        stop(): void {
            // Uncomment and adapt to your library:
            // motor.motorStop(this.leftPin);
            // motor.motorStop(this.rightPin);

            // For now, just show a message
            serial.writeLine("Motors stopped");
        }
    }

    /**
     * Create a servo driver instance
     * @param leftPin Left servo pin
     * @param rightPin Right servo pin
     * @param invertLeft Invert left servo direction
     * @param invertRight Invert right servo direction
     */
    //% block="Create servo driver with left pin %leftPin right pin %rightPin || invert left %invertLeft invert right %invertRight"
    //% leftPin.defl=AnalogPin.P1 rightPin.defl=AnalogPin.P2
    //% invertLeft.defl=false invertRight.defl=false
    //% expandableArgumentMode="toggle"
    export function createServoDriver(
        leftPin: AnalogPin,
        rightPin: AnalogPin,
        invertLeft = false,
        invertRight = false
    ): ServoDriver {
        return new ServoDriver(leftPin, rightPin, invertLeft, invertRight);
    }

    /**
     * Create a motor:bit driver instance
     * @param leftPin1 Left motor pin 1
     * @param leftPin2 Left motor pin 2
     * @param rightPin1 Right motor pin 1
     * @param rightPin2 Right motor pin 2
     */
    //% block="Create motor:bit driver with pins L1:%leftPin1 L2:%leftPin2 R1:%rightPin1 R2:%rightPin2"
    //% leftPin1.defl=DigitalPin.P0 leftPin2.defl=DigitalPin.P1
    //% rightPin1.defl=DigitalPin.P2 rightPin2.defl=DigitalPin.P8
    export function createMotorBitDriver(
        leftPin1: DigitalPin,
        leftPin2: DigitalPin,
        rightPin1: DigitalPin,
        rightPin2: DigitalPin
    ): MotorBitDriver {
        return new MotorBitDriver(leftPin1, leftPin2, rightPin1, rightPin2);
    }

    /**
     * Create a motor library adapter
     * @param leftPin Left motor pin
     * @param rightPin Right motor pin
     */
    //% block="Create motor library adapter with left pin %leftPin right pin %rightPin"
    //% leftPin.defl=0 rightPin.defl=1
    export function createMotorLibraryAdapter(
        leftPin: number,
        rightPin: number
    ): MotorLibraryAdapter {
        return new MotorLibraryAdapter(leftPin, rightPin);
    }
}

/**
 * Robot Movement Controller
 * Provides high-level movement controls for a differential-drive robot
 */
//% color="#00A1E9" weight=100 icon="\uf1b9" block="RobotMovement"
namespace RobotMovement {
    // PID constants for movement control
    const DEFAULT_KP = 0.5;   // Proportional gain
    const DEFAULT_KI = 0.05;  // Integral gain
    const DEFAULT_KD = 0.2;   // Derivative gain

    // Maximum time for any movement action (ms)
    const MOVEMENT_TIMEOUT = 10000;

    // Robot controller configuration
    let motorDriver: MotorDrivers.IMotorDriver = null;
    let isInitialized = false;
    let baseSpeed = 50;  // Default speed (0-100)

    // PID control variables
    let pidConfig = {
        kp: DEFAULT_KP,
        ki: DEFAULT_KI,
        kd: DEFAULT_KD
    };
    let previousError = 0;
    let integralError = 0;

    /**
     * Initialize the robot movement controller
     * @param motor Motor driver to use
     */
    //% block="Initialize robot with motor driver %motor=variables_get"
    //% weight=100
    export function initialize(motor: MotorDrivers.IMotorDriver): void {
        motorDriver = motor;
        isInitialized = true;

        // Reset everything
        resetPID();
        if (motorDriver) motorDriver.stop();

        // Show success/failure
        if (motorDriver) {
            basic.showIcon(IconNames.Yes);
            serial.writeLine("Robot movement system initialized successfully");
        } else {
            basic.showIcon(IconNames.No);
            serial.writeLine("Error: motor driver not properly set");
            isInitialized = false;
        }
    }

    /**
     * Set the base speed for movements
     * @param speed Speed value (0-100)
     */
    //% block="Set base speed to %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% weight=90
    export function setBaseSpeed(speed: number): void {
        baseSpeed = Math.constrain(speed, 0, 100);
    }

    /**
     * Configure PID parameters
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    //% block="Configure PID with Kp:%kp Ki:%ki Kd:%kd"
    //% kp.defl=0.5 ki.defl=0.05 kd.defl=0.2
    //% weight=85
    export function configurePID(kp: number, ki: number, kd: number): void {
        pidConfig.kp = kp;
        pidConfig.ki = ki;
        pidConfig.kd = kd;
        resetPID();
    }

    /**
     * Reset PID error values
     */
    function resetPID(): void {
        previousError = 0;
        integralError = 0;
    }

    /**
     * Check if the robot is initialized
     */
    function checkInitialized(): void {
        if (!isInitialized) {
            basic.showIcon(IconNames.No);
            serial.writeLine("Error: Robot not initialized");
        }
    }

    /**
     * Apply PID correction to an error value
     * @param error The current error to correct
     */
    function applyPID(error: number): number {
        // Calculate PID components
        const proportional = error * pidConfig.kp;
        integralError += error;
        const integral = integralError * pidConfig.ki;
        const derivative = (error - previousError) * pidConfig.kd;
        previousError = error;

        // Calculate total correction
        return proportional + integral + derivative;
    }

    /**
     * Calculate the smallest angle between two angles in radians
     * @param angle1 First angle in radians
     * @param angle2 Second angle in radians
     */
    function angleBetween(angle1: number, angle2: number): number {
        let diff = angle1 - angle2;
        return odometry.normalizeAngle(diff);
    }

    /**
     * Stop all motors
     */
    //% block="Stop motors"
    //% weight=80
    export function stop(): void {
        checkInitialized();
        if (motorDriver) {
            motorDriver.stop();
        }
    }

    /**
     * Move the robot forward by a specified distance
     * @param distance Distance to move in mm
     */
    //% block="Move forward %distance mm"
    //% distance.defl=100 distance.min=0
    //% weight=70
    export function moveForward(distance: number): void {
        checkInitialized();

        // Calculate target position
        const startX = odometry.getX();
        const startY = odometry.getY();
        const angle = odometry.getOrientationRad();
        const targetX = startX + Math.cos(angle) * distance;
        const targetY = startY + Math.sin(angle) * distance;

        // Reset PID control
        resetPID();

        // Start motors
        motorDriver.setMotorSpeed(
            baseSpeed, baseSpeed,
            MotorDrivers.Direction.Forward,
            MotorDrivers.Direction.Forward
        );

        // Movement control loop
        const startTime = input.runningTime();
        while (true) {
            // Note: Odometry is updated separately in a 50ms loop

            // Calculate remaining distance
            const distRemaining = odometry.distanceTo(targetX, targetY);

            // Check if we reached the target
            if (distRemaining < 5) { // 5mm precision
                break;
            }

            // Control angle to maintain straight line
            const currentAngle = odometry.getOrientationRad();
            const angleError = angleBetween(angle, currentAngle);

            // Apply PID control
            const correction = applyPID(angleError);

            // Speed control based on distance
            let speedFactor = 1.0;
            if (distRemaining < 100) {
                speedFactor = Math.max(distRemaining / 100, 0.3); // Min 30% speed
            }

            // Apply correction to motors
            const leftSpeed = (baseSpeed - correction) * speedFactor;
            const rightSpeed = (baseSpeed + correction) * speedFactor;

            motorDriver.setMotorSpeed(
                leftSpeed, rightSpeed,
                MotorDrivers.Direction.Forward,
                MotorDrivers.Direction.Forward
            );

            // Check timeout
            if (input.runningTime() - startTime > MOVEMENT_TIMEOUT) {
                serial.writeLine("Movement timeout");
                break;
            }

            // Short delay
            basic.pause(10);
        }

        // Stop motors
        stop();
    }

    /**
     * Turn the robot by a specified angle
     * @param angleDegrees Angle to turn in degrees (positive = counterclockwise)
     */
    //% block="Turn %angleDegrees degrees"
    //% angleDegrees.min=-180 angleDegrees.max=180 angleDegrees.defl=90
    //% weight=60
    export function turn(angleDegrees: number): void {
        checkInitialized();

        // Normalize angle to -180/+180
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;

        // Convert to radians
        const angleRadians = angleDegrees * Math.PI / 180;

        // Calculate target orientation
        const targetOrientation = odometry.getOrientationRad() + angleRadians;

        // Reset PID control
        resetPID();

        // Determine turn direction
        const turnDirection = angleDegrees > 0;
        const leftDir = turnDirection ? MotorDrivers.Direction.Backward : MotorDrivers.Direction.Forward;
        const rightDir = turnDirection ? MotorDrivers.Direction.Forward : MotorDrivers.Direction.Backward;

        // Start turn
        motorDriver.setMotorSpeed(baseSpeed, baseSpeed, leftDir, rightDir);

        // Movement control loop
        const startTime = input.runningTime();
        while (true) {
            // Note: Odometry is updated separately in a 50ms loop

            // Calculate error
            const currentOrientation = odometry.getOrientationRad();
            const error = angleBetween(targetOrientation, currentOrientation);

            // Check if we reached the target
            if (Math.abs(error) < 0.03) { // ~1.7 degrees precision
                break;
            }

            // Apply PID control
            const correction = applyPID(error);

            // Apply speeds based on turn direction
            let leftSpeed = baseSpeed + correction;
            let rightSpeed = baseSpeed - correction;

            // Ensure speeds are valid
            leftSpeed = Math.constrain(leftSpeed, 0, 100);
            rightSpeed = Math.constrain(rightSpeed, 0, 100);

            // Apply to motors
            motorDriver.setMotorSpeed(
                leftSpeed, rightSpeed,
                leftDir, rightDir
            );

            // Check timeout
            if (input.runningTime() - startTime > MOVEMENT_TIMEOUT) {
                serial.writeLine("Turn timeout");
                break;
            }

            // Short delay
            basic.pause(10);
        }

        // Stop motors
        stop();
    }

    /**
     * Move the robot to a relative position
     * @param x X distance in mm (forward is positive)
     * @param y Y distance in mm (left is positive)
     */
    //% block="Move to relative position X:%x Y:%y mm"
    //% x.defl=100 y.defl=0
    //% weight=50
    export function moveRelative(x: number, y: number): void {
        checkInitialized();

        // Skip if no movement needed
        if (x === 0 && y === 0) return;

        // Calculate target position in global coordinates
        const currentX = odometry.getX();
        const currentY = odometry.getY();
        const currentAngle = odometry.getOrientationRad();

        // Transform from robot-relative to global coordinates
        const cosAngle = Math.cos(currentAngle);
        const sinAngle = Math.sin(currentAngle);
        const targetX = currentX + x * cosAngle - y * sinAngle;
        const targetY = currentY + x * sinAngle + y * cosAngle;

        // Calculate required angle and distance
        const deltaX = targetX - currentX;
        const deltaY = targetY - currentY;
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        const targetAngle = Math.atan2(deltaY, deltaX);

        // Angle to turn
        const angleToTurn = angleBetween(targetAngle, currentAngle);
        const angleDegrees = angleToTurn * 180 / Math.PI;

        // First turn to face the target
        turn(angleDegrees);

        // Then move forward
        moveForward(distance);
    }

    /**
     * Example of how to set up a continuous odometry update
     * Call this from your main program, not used directly by RobotMovement
     * @param leftDeltaProvider Function to get left encoder delta (in ticks)
     * @param rightDeltaProvider Function to get right encoder delta (in ticks)
     */
    //% block="Start odometry update loop with %leftDeltaProvider and %rightDeltaProvider"
    //% draggableParameters="reporter"
    //% weight=40
    export function startOdometryUpdateLoop(
        leftDeltaProvider: () => number,
        rightDeltaProvider: () => number
    ): void {
        // Create a background loop to update odometry every 50ms
        control.inBackground(() => {
            while (true) {
                // Get deltas from encoders
                const leftDelta = leftDeltaProvider();
                const rightDelta = rightDeltaProvider();
                // Update odometry
                odometry.updateFromTicks(leftDelta, rightDelta);
                // Wait for next update cycle
                basic.pause(50);
            }
        });
    }
}

/**
 * Exemple simple d'utilisation du robot avec mise à jour d'odométrie
 */
/*
namespace Exemple {

    //% block="Exemple d'utilisation du robot"
    export function exempleUtilisation(): void {
        // Initialisation de l'odométrie
        odometry.initialize(150, 2000); // 150mm entre les roues, 2000 ticks par mètre

        // Création du pilote moteur (servomoteurs continus)
        let moteurs = MotorDrivers.createServoDriver(AnalogPin.P1, AnalogPin.P2);

        // Initialisation du contrôleur de mouvement
        RobotMovement.initialize(moteurs);

        // Configuration du PID (optionnel)
        RobotMovement.configurePID(0.5, 0.05, 0.2);

        // Définition de la vitesse de base
        RobotMovement.setBaseSpeed(60);

        // Fonction pour lire les encodeurs (à adapter selon votre hardware)
        let dernierTicksGauche = 0;
        let dernierTicksDroite = 0;

        // Démarrage de la mise à jour d'odométrie en arrière-plan
        RobotMovement.startOdometryUpdateLoop(
            // Fonction pour obtenir le delta de l'encodeur gauche
            () => {
                // Ceci est à adapter en fonction de votre système d'encodeurs
                // Par exemple en lisant des pins analogiques, I2C, etc.
                const nouveauxTicksGauche = pins.analogReadPin(AnalogPin.P3);
                const deltaGauche = nouveauxTicksGauche - dernierTicksGauche;
                dernierTicksGauche = nouveauxTicksGauche;
                return deltaGauche;
            },
            // Fonction pour obtenir le delta de l'encodeur droit
            () => {
                const nouveauxTicksDroite = pins.analogReadPin(AnalogPin.P4);
                const deltaDroite = nouveauxTicksDroite - dernierTicksDroite;
                dernierTicksDroite = nouveauxTicksDroite;
                return deltaDroite;
            }
        );

        // Exemple de séquence de mouvements
        basic.pause(1000);
        RobotMovement.moveForward(200); // Avance de 200mm
        basic.pause(500);
        RobotMovement.turn(90);         // Tourne de 90°
        basic.pause(500);
        RobotMovement.moveRelative(0, 100); // Déplacement latéral de 100mm
        basic.pause(500);
        RobotMovement.stop();
    }

    
}*/