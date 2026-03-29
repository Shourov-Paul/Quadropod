class Kinematics {
    static DIM = {
        L_COXA: 50.0,
        L_FEMUR: 70.0,
        L_TIBIA: 90.0
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
}
