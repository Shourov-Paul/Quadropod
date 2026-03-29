class Kinematics {
    static DIM = {
        L_COXA: 50.0,
        L_FEMUR: 70.0,
        L_TIBIA: 90.0,
        BODY_W: 120, // Body Width (X axis spanning left/right)
        BODY_L: 180  // Body Length (Y axis spanning front/back)
    };

    /**
     * Inverse Kinematics for a 3-DOF Quadruped Leg
     * @param {number} x - Forward/back relative to leg base
     * @param {number} y - Outward/inward distance (must be > 0 normally)
     * @param {number} z - Up/down (usually negative for downward reach)
     * @returns {Object} { hip, shoulder, knee } in degrees (assuming 90 is neutral)
     */
    static solveIK(x, y, z) {
        let L = Math.sqrt(x * x + y * y);
        let gamma = Math.atan2(y, x); // Hip angle in radians
        
        let L1 = L - this.DIM.L_COXA;
        if (L1 < 0) L1 = 0;
        
        let D = Math.sqrt(L1 * L1 + z * z);
        let maxReach = this.DIM.L_FEMUR + this.DIM.L_TIBIA;
        if (D > maxReach) {
            D = maxReach - 0.01; // Avoid NaN in acos due to floating point error
        }

        let alpha1 = Math.atan2(Math.abs(z), L1);
        let cosAlpha2 = (this.DIM.L_FEMUR ** 2 + D ** 2 - this.DIM.L_TIBIA ** 2) / (2 * this.DIM.L_FEMUR * D);
        let alpha2 = Math.acos(Math.max(-1, Math.min(1, cosAlpha2)));
        let alpha = alpha1 + alpha2; // Shoulder angle internal

        let cosBeta = (this.DIM.L_FEMUR ** 2 + this.DIM.L_TIBIA ** 2 - D ** 2) / (2 * this.DIM.L_FEMUR * this.DIM.L_TIBIA);
        let betaInner = Math.acos(Math.max(-1, Math.min(1, cosBeta)));
        let beta = Math.PI - betaInner; // Knee angle internal

        return {
            hip: 90 + (gamma * 180 / Math.PI),
            shoulder: 90 + (alpha * 180 / Math.PI),
            knee: 90 + (beta * 180 / Math.PI)
        };
    }

    /**
     * Forward Kinematics to calculate actual 3D joint positions
     * @param {number} legIndex - 0: FL, 1: FR, 2: BL, 3: BR
     * @param {number} hipDeg - Hip angle in degrees
     * @param {number} shoulderDeg - Shoulder angle in degrees
     * @param {number} kneeDeg - Knee angle in degrees
     * @returns {Array} Array of 4 points: [BodyMount, CoxaEnd, FemurEnd, TibiaEnd] each as {x,y,z}
     */
    static solveFK(legIndex, hipDeg, shoulderDeg, kneeDeg) {
        // Convert to radians and offset (90 is neutral)
        let gamma = (hipDeg - 90) * (Math.PI / 180);
        let alpha = -(shoulderDeg - 90) * (Math.PI / 180); // Invert to match Z-up
        let beta = -(kneeDeg - 90) * (Math.PI / 180);

        // Calculate positions in 2D plane originating from the leg mount
        // r = distance extending outwards
        // z = distance up/down
        
        let p0 = { r: 0, z: 0 };
        
        let p1 = {
            r: this.DIM.L_COXA,
            z: 0
        };

        let p2 = {
            r: p1.r + this.DIM.L_FEMUR * Math.cos(alpha),
            z: p1.z + this.DIM.L_FEMUR * Math.sin(alpha)
        };

        // The tibia bends relative to the femur
        let p3 = {
            r: p2.r + this.DIM.L_TIBIA * Math.cos(alpha - beta),
            z: p2.z + this.DIM.L_TIBIA * Math.sin(alpha - beta)
        };

        // Array of local 2D joints
        const localJoints = [p0, p1, p2, p3];

        // Map to 3D Space (X=Forward, Y=Left, Z=Up)
        // Leg Mount Definitions:
        // 0 (FL): +X, +Y, Angle: 45 deg (PI/4)
        // 1 (FR): +X, -Y, Angle: -45 deg (-PI/4)
        // 2 (BL): -X, +Y, Angle: 135 deg (3*PI/4)
        // 3 (BR): -X, -Y, Angle: -135 deg (-3*PI/4)
        
        const mountOffsets = [
            { x: this.DIM.BODY_L / 2, y: this.DIM.BODY_W / 2, baseAngle: Math.PI / 4 },     // FL
            { x: this.DIM.BODY_L / 2, y: -this.DIM.BODY_W / 2, baseAngle: -Math.PI / 4 },    // FR
            { x: -this.DIM.BODY_L / 2, y: this.DIM.BODY_W / 2, baseAngle: 3 * Math.PI / 4 }, // BL
            { x: -this.DIM.BODY_L / 2, y: -this.DIM.BODY_W / 2, baseAngle: -3 * Math.PI / 4 }// BR
        ];

        const mount = mountOffsets[legIndex];
        const totalAngle = mount.baseAngle + gamma;

        return localJoints.map(joint => {
            return {
                x: mount.x + joint.r * Math.cos(totalAngle),
                y: mount.y + joint.r * Math.sin(totalAngle),
                z: (mount.z || 0) + joint.z
            };
        });
    }
}
