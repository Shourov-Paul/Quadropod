class RobotViewer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);

        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.fog = new THREE.FogExp2(0x0a0a0f, 0.002);

        // Camera setup
        this.camera = new THREE.PerspectiveCamera(45, this.container.clientWidth / this.container.clientHeight, 0.1, 1000);
        this.camera.position.set(250, 150, 250);

        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setClearColor(0x0a0a0f, 0); // Transparent background to show CSS
        this.container.appendChild(this.renderer.domElement);

        // Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.maxPolarAngle = Math.PI / 2 - 0.01; // Don't go below ground

        // Robot parts references
        this.legs = []; // Array of 4 legs, each containing { coxaPivot, femurPivot, tibiaPivot }

        this.setupLighting();
        this.buildRobot();

        // Handle resizing
        window.addEventListener('resize', this.onWindowResize.bind(this), false);

        // Start loop
        this.animate = this.animate.bind(this);
        requestAnimationFrame(this.animate);
    }

    setupLighting() {
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
        this.scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0x00f0ff, 0.8);
        dirLight.position.set(100, 200, 100);
        this.scene.add(dirLight);

        const backLight = new THREE.PointLight(0xff3366, 0.6, 500);
        backLight.position.set(-100, 50, -100);
        this.scene.add(backLight);

        // Floor grid
        const gridHelper = new THREE.GridHelper(600, 30, 0x00f0ff, 0x222222);
        gridHelper.position.y = -120; // Represents the ground
        gridHelper.material.opacity = 0.3;
        gridHelper.material.transparent = true;
        this.scene.add(gridHelper);
    }

    buildRobot() {
        // Materials
        const bodyMat = new THREE.MeshPhongMaterial({ color: 0x222222, specular: 0x555555, shininess: 30 });
        const jointMat = new THREE.MeshPhongMaterial({ color: 0x00f0ff });
        const boneMat = new THREE.MeshPhongMaterial({ color: 0xdddddd });

        // Body Dimensions
        const BODY_W = 120; // Width X
        const BODY_L = 180; // Length Z
        const BODY_H = 40;  // Height Y

        // Main Body Box
        const bodyGeo = new THREE.BoxGeometry(BODY_W, BODY_H, BODY_L);
        this.robotBody = new THREE.Mesh(bodyGeo, bodyMat);
        this.scene.add(this.robotBody);

        // Define leg mounting positions (Front/Back, Left/Right)
        // Leg 1: FL, Leg 2: FR, Leg 3: BL, Leg 4: BR
        const mountOffsets = [
            { x: BODY_W / 2, z: BODY_L / 2, angle: Math.PI / 4 },     // FL (Leg 1)
            { x: -BODY_W / 2, z: BODY_L / 2, angle: 3 * Math.PI / 4 },  // FR (Leg 2)
            { x: BODY_W / 2, z: -BODY_L / 2, angle: -Math.PI / 4 },   // BL (Leg 3)
            { x: -BODY_W / 2, z: -BODY_L / 2, angle: -3 * Math.PI / 4 } // BR (Leg 4)
        ];

        // Create 4 legs
        for (let i = 0; i < 4; i++) {
            const mount = mountOffsets[i];

            // Base Mount Assembly
            const legBase = new THREE.Group();
            legBase.position.set(mount.x, 0, mount.z);
            legBase.rotation.y = mount.angle;
            this.robotBody.add(legBase);

            // -- Hip (Coxa) --
            // Pivot rotates around Y axis
            const coxaPivot = new THREE.Group();
            legBase.add(coxaPivot);

            // Generate visuals for Coxa
            const coxaGeo = new THREE.BoxGeometry(Kinematics.DIM.L_COXA, 15, 20);
            const coxaMesh = new THREE.Mesh(coxaGeo, boneMat);
            coxaMesh.position.set(Kinematics.DIM.L_COXA / 2, 0, 0); // Offset to rotate from end
            coxaPivot.add(coxaMesh);

            // Joint marker
            const joint1 = new THREE.Mesh(new THREE.CylinderGeometry(12, 12, 16, 16), jointMat);
            coxaPivot.add(joint1);

            // -- Shoulder (Femur) --
            // Pivot rotates around Z axis (lifting leg up/down)
            const femurPivot = new THREE.Group();
            femurPivot.position.set(Kinematics.DIM.L_COXA, 0, 0); // At end of Coxa
            coxaPivot.add(femurPivot);

            const femurGeo = new THREE.BoxGeometry(Kinematics.DIM.L_FEMUR, 12, 16);
            const femurMesh = new THREE.Mesh(femurGeo, boneMat);
            femurMesh.position.set(Kinematics.DIM.L_FEMUR / 2, 0, 0);
            femurPivot.add(femurMesh);

            const joint2 = new THREE.Mesh(new THREE.CylinderGeometry(10, 10, 18, 16), jointMat);
            joint2.rotation.x = Math.PI / 2;
            femurPivot.add(joint2);

            // -- Knee (Tibia) --
            // Pivot rotates around Z axis
            const tibiaPivot = new THREE.Group();
            tibiaPivot.position.set(Kinematics.DIM.L_FEMUR, 0, 0); // At end of Femur
            femurPivot.add(tibiaPivot);

            const tibiaGeo = new THREE.BoxGeometry(Kinematics.DIM.L_TIBIA, 10, 12);
            const tibiaMesh = new THREE.Mesh(tibiaGeo, boneMat);
            tibiaMesh.position.set(Kinematics.DIM.L_TIBIA / 2, 0, 0);
            tibiaPivot.add(tibiaMesh);

            const joint3 = new THREE.Mesh(new THREE.CylinderGeometry(8, 8, 14, 16), jointMat);
            joint3.rotation.x = Math.PI / 2;
            tibiaPivot.add(joint3);

            // Save pivots for animation
            this.legs.push({
                coxa: coxaPivot,
                femur: femurPivot,
                tibia: tibiaPivot
            });

            // Set neutral position (90 degrees equivalent based on kinematic model)
            // The kinematics expects 90 to be neutral. We will map 0-180 inputs to radians.
        }
    }

    /**
     * Updates the visuals of a specific leg
     * @param {number} legIndex 0-3
     * @param {number} hipAngle 0-180
     * @param {number} shoulderAngle 0-180
     * @param {number} kneeAngle 0-180
     */
    updateLegJoints(legIndex, hipAngle, shoulderAngle, kneeAngle) {
        if (legIndex < 0 || legIndex > 3) return;

        const leg = this.legs[legIndex];

        // Map 0-180 degrees from servo commands to model radians
        // Assuming 90 is neutral straight out

        // Hip (Coxa): Rotate around Y
        // 90 is straight. <90 moves backward, >90 moves forward
        leg.coxa.rotation.y = (hipAngle - 90) * (Math.PI / 180);

        // Shoulder (Femur): Rotate around Z
        // 90 is horizontal. <90 is pointing down, >90 is pointing up (or vice versa)
        leg.femur.rotation.z = (shoulderAngle - 90) * (Math.PI / 180);

        // Knee (Tibia): Rotate around Z relative to Femur
        // 90 is straight line extending from femur.
        // Usually knee bends downwards.
        leg.tibia.rotation.z = -(kneeAngle - 90) * (Math.PI / 180);
    }

    onWindowResize() {
        if (!this.container || !this.camera || !this.renderer) return;
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }

    animate() {
        requestAnimationFrame(this.animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    resetView() {
        this.camera.position.set(250, 150, 250);
        this.controls.target.set(0, 0, 0);
    }
}
