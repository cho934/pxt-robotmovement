
/**
 * Namespace pour le contrôle du déplacement d'un robot microbit
 * Utilise l'odométrie et les encodeurs magnétiques pour le positionnement précis
 * Avec support de différents drivers moteurs via des callbacks
 */
//% color="#00A1E9" weight=100 icon="\uf1b9" block="RobotMovement"
namespace RobotMovement {
    // Définition des types pour les callbacks
    export type MotorDriverFunction = (leftSpeed: number, rightSpeed: number, leftDirection: Direction, rightDirection: Direction) => void;
    export type EncoderReaderFunction = () => { leftDelta: number, rightDelta: number };

    // Enum pour la direction des moteurs
    export enum Direction {
        //% block="Avant (CW)"
        Forward = 0,
        //% block="Arrière (CCW)"
        Backward = 1
    }

    // Variables pour les moteurs et encodeurs
    let isInitialized = false;
    let baseSpeed = 50; // Vitesse de base des moteurs (0-100)

    // Callbacks pour les fonctions adaptables
    let motorDriverCallback: MotorDriverFunction = null;
    let encoderReaderCallback: EncoderReaderFunction = null;

    // Constantes PID
    const Kp = 0.5;    // Coefficient proportionnel
    const Ki = 0.05;   // Coefficient intégral
    const Kd = 0.2;    // Coefficient dérivé

    // Variables pour garder une trace de l'erreur précédente et de l'erreur cumulée
    let previousError = 0;
    let integralError = 0;

    // Timeout pour les mouvements (ms)
    const MOVEMENT_TIMEOUT = 10000;

    /**
     * Définit le gestionnaire de moteur pour les moteurs standard
     * @param pin1 Pin moteur gauche
     * @param pin2 Pin moteur droit
     */
    //% blockId=robotmovement_set_standard_motors
    //% block="Utiliser moteurs standard sur pins %pin1 et %pin2"
    //% pin1.defl=0 pin2.defl=1
    //% weight=100
    /*
    export function setStandardMotors(pin1: number, pin2: number): void {
        initialize(
            // Callback pour les moteurs
            (leftSpeed, rightSpeed, leftDir, rightDir) => {
                // Pour les moteurs utilisant la librairie "motor"
                const leftDirMap = leftDir === Direction.Forward ? motor.Dir.CW : motor.Dir.CCW;
                const rightDirMap = rightDir === Direction.Forward ? motor.Dir.CW : motor.Dir.CCW;
                motor.motorRun(pin1, leftDirMap, leftSpeed);
                motor.motorRun(pin2, rightDirMap, rightSpeed);
            },
            // Callback pour les encodeurs
            () => {
                // Exemple avec les encodeurs magnétiques
                encoders.getValues();
                return {
                    leftDelta: encoders.getDeltaLeftValue(),
                    rightDelta: encoders.getDeltaRightValue()
                };
            }
        );
    }*/

    /**
     * Définit le gestionnaire de moteur pour les servomoteurs continus
     * @param pinLeft Pin servo moteur gauche
     * @param pinRight Pin servo moteur droit
     */
    //% blockId=robotmovement_set_servos
    //% block="Utiliser servomoteurs continus sur pins %pinLeft et %pinRight"
    //% pinLeft.defl=AnalogPin.P1 pinRight.defl=AnalogPin.P2
    //% weight=90
    /*
    export function setContinuousServos(pinLeft: AnalogPin, pinRight: AnalogPin): void {
        initialize(
            // Callback pour les servomoteurs
            (leftSpeed, rightSpeed, leftDir, rightDir) => {
                // Pour des servomoteurs continus:
                // - La valeur 90 correspond à l'arrêt
                // - Les valeurs 0-89 => rotation dans un sens
                // - Les valeurs 91-180 => rotation dans l'autre sens

                let leftValue = 90;  // Valeur par défaut = arrêt
                let rightValue = 90; // Valeur par défaut = arrêt

                if (leftSpeed > 0) {
                    if (leftDir === Direction.Forward) {
                        leftValue = 90 - Math.map(leftSpeed, 0, 100, 0, 90);
                    } else {
                        leftValue = 90 + Math.map(leftSpeed, 0, 100, 0, 90);
                    }
                }

                if (rightSpeed > 0) {
                    if (rightDir === Direction.Forward) {
                        rightValue = 90 + Math.map(rightSpeed, 0, 100, 0, 90);
                    } else {
                        rightValue = 90 - Math.map(rightSpeed, 0, 100, 0, 90);
                    }
                }

                // Limiter les valeurs dans la plage 0-180
                leftValue = Math.constrain(leftValue, 0, 180);
                rightValue = Math.constrain(rightValue, 0, 180);

                // Envoyer les commandes aux servomoteurs
                pins.servoWritePin(pinLeft, leftValue);
                pins.servoWritePin(pinRight, rightValue);

                // Afficher les valeurs pour débogage (optionnel)
                serial.writeString("Servo G: " + leftValue + " D: " + rightValue + "\n");
            },

            // Callback pour les encodeurs (exemple avec AS5048B)
            () => {
                ams_AS5048B.update();
                return {
                    leftDelta: ams_AS5048B.getDeltaLeftTicks(),
                    rightDelta: ams_AS5048B.getDeltaRightTicks()
                };
            }
        );
    }*/

    /**
     * Définit des gestionnaires personnalisés pour les moteurs et encodeurs
     * @param onMotorControl Fonction de contrôle des moteurs
     * @param onEncoderRead Fonction de lecture des encodeurs
     */
    //% blockId=robotmovement_set_custom_handlers
    //% block="Définir gestionnaires personnalisés"
    //% draggableParameters="reporter"
    //% weight=80
    export function setCustomHandlers(
        onMotorControl: (leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction) => void,
        onEncoderRead: () => { leftDelta: number, rightDelta: number }
    ): void {
        initialize(onMotorControl, onEncoderRead);
    }

    /**
     * Initialise le système de déplacement avec des callbacks adaptables
     * @param motorDriver Fonction pour contrôler les moteurs
     * @param encoderReader Fonction pour lire les valeurs des encodeurs
     */
    function initialize(motorDriver: MotorDriverFunction, encoderReader: EncoderReaderFunction): void {
        motorDriverCallback = motorDriver;
        encoderReaderCallback = encoderReader;
        isInitialized = true;

        // Vérification de l'initialisation
        if (motorDriverCallback && encoderReaderCallback) {
            basic.showIcon(IconNames.Yes);
            serial.writeLine("Système de déplacement initialisé");
        } else {
            basic.showIcon(IconNames.No);
            serial.writeLine("Erreur: callbacks non définis");
            isInitialized = false;
        }
    }

    /**
     * Définit la vitesse de base des moteurs
     * @param speed Vitesse entre 0 et 100
     */
    //% blockId=robotmovement_set_base_speed
    //% block="Régler vitesse de base à %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% weight=70
    export function setBaseSpeed(speed: number): void {
        baseSpeed = Math.constrain(speed, 0, 100);
    }

    /**
     * Applique les commandes de moteur en utilisant le callback défini
     * @param leftSpeed Vitesse du moteur gauche (0-100)
     * @param rightSpeed Vitesse du moteur droit (0-100)
     * @param leftDir Direction du moteur gauche
     * @param rightDir Direction du moteur droit
     */
    function applyMotorCommand(leftSpeed: number, rightSpeed: number, leftDir: Direction, rightDir: Direction): void {
        if (motorDriverCallback) {
            // Limiter les vitesses entre 0 et 100
            leftSpeed = Math.constrain(leftSpeed, 0, 100);
            rightSpeed = Math.constrain(rightSpeed, 0, 100);

            // Appeler le callback moteur
            motorDriverCallback(leftSpeed, rightSpeed, leftDir, rightDir);
        }
    }

    /**
     * Lit les valeurs des encodeurs en utilisant le callback défini
     * @returns Objet contenant les deltas des encodeurs gauche et droit
     */
    function readEncoders(): { leftDelta: number, rightDelta: number } {
        if (encoderReaderCallback) {
            return encoderReaderCallback();
        }
        return { leftDelta: 0, rightDelta: 0 };
    }

    /**
     * Arrête les moteurs
     */
    //% blockId=robotmovement_stop_motors
    //% block="Arrêter les moteurs"
    //% weight=60
    export function stopMotors(): void {
        if (motorDriverCallback) {
            motorDriverCallback(0, 0, Direction.Forward, Direction.Forward);
        }
    }

    /**
     * Déplace le robot vers une position relative (x,y) en mm
     * @param distX distance à parcourir en x (mm)
     * @param distY distance à parcourir en y (mm)
     */
    //% blockId=robotmovement_movexy
    //% block="Déplacer de X:%distX mm Y:%distY mm"
    //% distX.defl=100 distY.defl=0
    //% weight=50
    export function movexy(distX: number, distY: number): void {
        if (!isInitialized) {
            serial.writeLine("Erreur: système de déplacement non initialisé");
            return;
        }

        // Calcul de la distance totale à parcourir
        const distance = Math.sqrt(distX * distX + distY * distY);

        // Calcul de l'angle cible en radians
        let targetAngle = Math.atan2(distY, distX);

        // Angle de rotation nécessaire (différence entre l'angle cible et l'angle actuel)
        let angleDiff = targetAngle - odometry.getOrientationRad();

        // Normalisation entre -π et π
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

        // Conversion en degrés pour la fonction turn
        let angleDiffDeg = angleDiff * 180 / Math.PI;

        // 1. Tourner vers la direction cible
        turn(angleDiffDeg);

        // 2. Avancer de la distance calculée
        moveForward(distance);
    }

    /**
     * Tourne le robot d'un angle spécifié en degrés
     * @param angleDegrees angle de rotation en degrés (positif = sens anti-horaire)
     */
    //% blockId=robotmovement_turn
    //% block="Tourner de %angleDegrees degrés"
    //% angleDegrees.min=-180 angleDegrees.max=180 angleDegrees.defl=90
    //% weight=40
    export function turn(angleDegrees: number): void {
        if (!isInitialized) {
            serial.writeLine("Erreur: système de déplacement non initialisé");
            return;
        }

        // Normaliser l'angle entre -180 et 180 degrés pour prendre le chemin le plus court
        while (angleDegrees > 180) angleDegrees -= 360;
        while (angleDegrees < -180) angleDegrees += 360;

        // Convertir en radians
        const angleRadians = angleDegrees * Math.PI / 180;

        // Position cible
        const targetOrientation = odometry.getOrientationRad() + angleRadians;

        // Réinitialiser les erreurs PID
        previousError = 0;
        integralError = 0;

        // Démarrer les moteurs dans la bonne direction
        if (angleDegrees > 0) {
            // Tourner à gauche (sens anti-horaire)
            applyMotorCommand(baseSpeed, baseSpeed, Direction.Backward, Direction.Forward);
        } else {
            // Tourner à droite (sens horaire)
            applyMotorCommand(baseSpeed, baseSpeed, Direction.Forward, Direction.Backward);
        }

        // Durée maximale pour éviter une boucle infinie
        const startTime = input.runningTime();

        // Boucle jusqu'à atteindre l'orientation cible
        while (true) {
            // Mettre à jour l'odométrie
            const encoderData = readEncoders();
            odometry.updateFromTicks(encoderData.leftDelta, encoderData.rightDelta);

            // Calculer l'erreur actuelle (différence entre l'orientation cible et l'orientation actuelle)
            let currentOrientation = odometry.getOrientationRad();
            let error = angleBetween(targetOrientation, currentOrientation);

            // Vérifier si l'objectif est atteint
            if (Math.abs(error) < 0.03) { // ~1.7 degrés de précision
                break;
            }

            // Calculer les composantes PID
            let proportional = error * Kp;
            integralError += error;
            let integral = integralError * Ki;
            let derivative = (error - previousError) * Kd;
            previousError = error;

            // Calculer la correction de la vitesse
            let correction = proportional + integral + derivative;

            // Appliquer la correction aux moteurs
            let leftSpeed = baseSpeed + correction;
            let rightSpeed = baseSpeed - correction;

            // Limiter les vitesses
            leftSpeed = Math.constrain(leftSpeed, 0, 100);
            rightSpeed = Math.constrain(rightSpeed, 0, 100);

            // Actualiser les vitesses des moteurs
            if (angleDegrees > 0) {
                // Tourner à gauche
                applyMotorCommand(leftSpeed, rightSpeed, Direction.Backward, Direction.Forward);
            } else {
                // Tourner à droite
                applyMotorCommand(leftSpeed, rightSpeed, Direction.Forward, Direction.Backward);
            }

            // Vérifier le timeout
            if (input.runningTime() - startTime > MOVEMENT_TIMEOUT) {
                serial.writeLine("Timeout de rotation");
                break;
            }

            // Petit délai pour éviter une utilisation trop intensive du processeur
            basic.pause(10);
        }

        // Arrêter les moteurs
        stopMotors();
    }

    /**
     * Déplace le robot vers l'avant d'une distance spécifiée en mm
     * @param distance distance à parcourir en mm
     */
    //% blockId=robotmovement_move_forward
    //% block="Avancer de %distance mm"
    //% distance.defl=100 distance.min=0
    //% weight=30
    export function moveForward(distance: number): void {
        if (!isInitialized) {
            serial.writeLine("Erreur: système de déplacement non initialisé");
            return;
        }

        // Calculer la position cible
        const startX = odometry.getX();
        const startY = odometry.getY();
        const angle = odometry.getOrientationRad();
        const targetX = startX + Math.cos(angle) * distance;
        const targetY = startY + Math.sin(angle) * distance;

        // Réinitialiser les erreurs PID
        previousError = 0;
        integralError = 0;

        // Démarrer les moteurs en marche avant
        applyMotorCommand(baseSpeed, baseSpeed, Direction.Forward, Direction.Forward);

        // Durée maximale pour éviter une boucle infinie
        const startTime = input.runningTime();

        // Boucle jusqu'à atteindre la position cible
        while (true) {
            // Mettre à jour l'odométrie
            const encoderData = readEncoders();
            odometry.updateFromTicks(encoderData.leftDelta, encoderData.rightDelta);

            // Calculer l'erreur de distance restante
            const distanceRemaining = odometry.distanceTo(targetX, targetY);

            // Calculer l'erreur d'orientation (pour rester en ligne droite)
            const currentAngle = odometry.getOrientationRad();
            const angleError = angleBetween(angle, currentAngle);

            // Vérifier si l'objectif est atteint
            if (distanceRemaining < 5) { // 5mm de précision
                break;
            }

            // Calculer la correction d'orientation par PID
            let proportional = angleError * Kp;
            integralError += angleError;
            let integral = integralError * Ki;
            let derivative = (angleError - previousError) * Kd;
            previousError = angleError;

            // Calculer la correction de la vitesse pour maintenir la trajectoire
            let correction = proportional + integral + derivative;

            // Appliquer la correction aux moteurs
            let leftSpeed = baseSpeed - correction;
            let rightSpeed = baseSpeed + correction;

            // Ajuster la vitesse globale en fonction de la distance restante
            let speedMultiplier = 1.0;
            if (distanceRemaining < 100) {
                speedMultiplier = distanceRemaining / 100;
                speedMultiplier = Math.max(speedMultiplier, 0.3); // Au moins 30% de la vitesse
            }

            leftSpeed *= speedMultiplier;
            rightSpeed *= speedMultiplier;

            // Actualiser les vitesses des moteurs
            applyMotorCommand(leftSpeed, rightSpeed, Direction.Forward, Direction.Forward);

            // Vérifier le timeout
            if (input.runningTime() - startTime > MOVEMENT_TIMEOUT) {
                serial.writeLine("Timeout de déplacement");
                break;
            }

            // Petit délai pour éviter une utilisation trop intensive du processeur
            basic.pause(10);
        }

        // Arrêter les moteurs
        stopMotors();
    }

    /**
     * Calcule la plus petite différence angulaire entre deux angles en radians
     * @param angle1 Premier angle en radians
     * @param angle2 Deuxième angle en radians
     * @returns Différence d'angle normalisée entre -π et π
     */
    function angleBetween(angle1: number, angle2: number): number {
        let diff = angle1 - angle2;

        // Normaliser entre -π et π
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        return diff;
    }

}