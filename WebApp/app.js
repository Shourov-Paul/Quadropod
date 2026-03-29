document.addEventListener('DOMContentLoaded', () => {
    // --- 3D Viewer Initialization ---
    const viewer = new RobotViewer('viewer-container');

    // --- State & Memory ---
    const state = {
        connected: false,
        port: null,
        writer: null,
        reader: null,
        angles: [
            { hip: 90, shoulder: 90, knee: 90 }, // Leg 1
            { hip: 90, shoulder: 90, knee: 90 }, // Leg 2
            { hip: 90, shoulder: 90, knee: 90 }, // Leg 3
            { hip: 90, shoulder: 90, knee: 90 }  // Leg 4
        ]
    };

    // --- DOM Elements ---
    const UI = {
        btnConnect: document.getElementById('btn-connect'),
        btnDisconnect: document.getElementById('btn-disconnect'),
        statusDot: document.getElementById('connection-status'),
        statusText: document.getElementById('connection-text'),
        console: document.getElementById('serial-console'),

        // Tabs
        tabBtns: document.querySelectorAll('.tab-btn'),
        tabContents: document.querySelectorAll('.tab-content'),

        // Calibration
        calibLeg: document.getElementById('calib-leg'),
        calibJoint: document.getElementById('calib-joint'),
        calibSlider: document.getElementById('calib-slider'),
        calibValDisplay: document.getElementById('calib-valDisplay'),
        btnSaveCalib: document.getElementById('btn-save-calib'),

        // Manual Control
        manualLeg: document.getElementById('manual-leg'),
        sliderHip: document.getElementById('slider-hip'),
        sliderShoulder: document.getElementById('slider-shoulder'),
        sliderKnee: document.getElementById('slider-knee'),
        valHip: document.getElementById('val-hip'),
        valShoulder: document.getElementById('val-shoulder'),
        valKnee: document.getElementById('val-knee'),

        // IK Control
        ikLeg: document.getElementById('ik-leg'),
        sliderX: document.getElementById('slider-x'),
        sliderY: document.getElementById('slider-y'),
        sliderZ: document.getElementById('slider-z'),
        valX: document.getElementById('val-x'),
        valY: document.getElementById('val-y'),
        valZ: document.getElementById('val-z'),

        // Gait Built-ins
        btnStand: document.getElementById('btn-stand'),
        btnSit: document.getElementById('btn-sit'),
        btnRest: document.getElementById('btn-rest'),

        // Directional controls
        btnFwd: document.getElementById('btn-fwd'),
        btnBack: document.getElementById('btn-back'),
        btnLeft: document.getElementById('btn-left'),
        btnRight: document.getElementById('btn-right'),
        btnStop: document.getElementById('btn-stop'),
        sliderSpeed: document.getElementById('slider-speed'),

        btnResetView: document.getElementById('btn-reset-view')
    };

    // --- Serial Communication ---
    function logToConsole(msg, type = 'tx') {
        const span = document.createElement('span');
        span.className = type;
        span.textContent = msg + '\n';
        UI.console.appendChild(span);
        UI.console.scrollTop = UI.console.scrollHeight;
    }

    async function connectSerial() {
        try {
            state.port = await navigator.serial.requestPort();
            await state.port.open({ baudRate: 115200 });

            state.connected = true;
            UI.btnConnect.classList.add('hidden');
            UI.btnDisconnect.classList.remove('hidden');
            UI.statusDot.className = 'status-dot connected';
            UI.statusText.textContent = 'Connected';

            logToConsole('Port opened successfully.', 'rx');

            const textEncoder = new TextEncoderStream();
            const writableStreamClosed = textEncoder.readable.pipeTo(state.port.writable);
            state.writer = textEncoder.writable.getWriter();

            // Start reading (non-blocking)
            readLoop();

            // Request current saved state from Arduino
            sendCommand('REPORT');

        } catch (err) {
            logToConsole('Error: ' + err.message, 'err');
        }
    }

    async function disconnectSerial() {
        state.connected = false;
        if (state.reader) await state.reader.cancel();
        if (state.writer) await state.writer.close();
        if (state.port) await state.port.close();

        UI.btnConnect.classList.remove('hidden');
        UI.btnDisconnect.classList.add('hidden');
        UI.statusDot.className = 'status-dot disconnected';
        UI.statusText.textContent = 'Disconnected';
        logToConsole('Port closed.', 'rx');
    }

    async function sendCommand(cmd) {
        if (!state.connected || !state.writer) return;
        await state.writer.write(cmd + '\n');
        logToConsole(`> ${cmd}`, 'tx');
    }

    async function readLoop() {
        while (state.port.readable && state.connected) {
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = state.port.readable.pipeTo(textDecoder.writable);
            state.reader = textDecoder.readable.getReader();

            try {
                while (true) {
                    const { value, done } = await state.reader.read();
                    if (done) break;
                    if (value) {
                        const lines = value.split('\n');
                        lines.forEach(line => {
                            line = line.trim();
                            if (line) {
                                logToConsole(`< ${line}`, 'rx');

                                // Parse inbound state reporting
                                if (line.startsWith('STATE:')) {
                                    const parts = line.split(':');
                                    if (parts[1] === 'DONE') {
                                        // Finished receiving state, sync active leg IK sliders
                                        syncManualSliders(parseInt(UI.manualLeg.value) - 1);
                                        // Recalculate IK visuals based on current angles if needed, 
                                        // or just let them be manually. This ensures the UI is flush with EEPROM.
                                    } else if (parts.length === 4) {
                                        const legIdx = parseInt(parts[1]) - 1;
                                        const jointIdx = parseInt(parts[2]);
                                        const angle = parseFloat(parts[3]);

                                        if (legIdx >= 0 && legIdx < 4) {
                                            if (jointIdx === 1) state.angles[legIdx].hip = angle;
                                            if (jointIdx === 2) state.angles[legIdx].shoulder = angle;
                                            if (jointIdx === 3) state.angles[legIdx].knee = angle;
                                            update3DModel(legIdx);
                                        }
                                    }
                                }
                            }
                        });
                    }
                }
            } catch (error) {
                logToConsole('Read error: ' + error, 'err');
            } finally {
                state.reader.releaseLock();
            }
        }
    }

    // --- Event Listeners ---
    UI.btnConnect.addEventListener('click', connectSerial);
    UI.btnDisconnect.addEventListener('click', disconnectSerial);
    UI.btnResetView.addEventListener('click', () => viewer.resetView());

    // Tab Switching
    UI.tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            UI.tabBtns.forEach(b => b.classList.remove('active'));
            UI.tabContents.forEach(c => c.classList.add('hidden'));

            btn.classList.add('active');
            document.getElementById(btn.dataset.target).classList.remove('hidden');
            document.getElementById(btn.dataset.target).classList.add('active');
        });
    });

    // Model Updating
    function update3DModel(legIndex) {
        const ag = state.angles[legIndex];
        viewer.updateLegJoints(legIndex, ag.hip, ag.shoulder, ag.knee);
    }

    function syncManualSliders(legIndex) {
        const ag = state.angles[legIndex];
        UI.sliderHip.value = ag.hip;
        UI.sliderShoulder.value = ag.shoulder;
        UI.sliderKnee.value = ag.knee;
        UI.valHip.textContent = Math.round(ag.hip);
        UI.valShoulder.textContent = Math.round(ag.shoulder);
        UI.valKnee.textContent = Math.round(ag.knee);
    }

    // Manual Controls
    UI.manualLeg.addEventListener('change', (e) => syncManualSliders(parseInt(e.target.value) - 1));

    function handleManualMove(jointName, index, sliderElement, valElement) {
        sliderElement.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            valElement.textContent = val;

            const legIdx = parseInt(UI.manualLeg.value) - 1;
            state.angles[legIdx][jointName] = val;

            update3DModel(legIdx);
            sendCommand(`L${legIdx + 1}S${index + 1}:${val}`);
        });
    }

    handleManualMove('hip', 0, UI.sliderHip, UI.valHip);
    handleManualMove('shoulder', 1, UI.sliderShoulder, UI.valShoulder);
    handleManualMove('knee', 2, UI.sliderKnee, UI.valKnee);

    // IK Controls
    function handleIKMove() {
        const x = parseFloat(UI.sliderX.value);
        const y = parseFloat(UI.sliderY.value);
        const z = parseFloat(UI.sliderZ.value);

        UI.valX.textContent = x;
        UI.valY.textContent = y;
        UI.valZ.textContent = z;

        const legIdx = parseInt(UI.ikLeg.value) - 1;

        // Send XYZ to ESP32 format: MOVE:1:X,Y,Z
        sendCommand(`MOVE:${legIdx + 1}:${x},${y},${z}`);

        // Update local 3D model
        const angles = Kinematics.solveIK(x, y, z);
        state.angles[legIdx].hip = angles.hip;
        state.angles[legIdx].shoulder = angles.shoulder;
        state.angles[legIdx].knee = angles.knee;

        // Sync manual sliders if we switch tabs
        syncManualSliders(legIdx);
        update3DModel(legIdx);
    }

    UI.sliderX.addEventListener('input', handleIKMove);
    UI.sliderY.addEventListener('input', handleIKMove);
    UI.sliderZ.addEventListener('input', handleIKMove);

    // Calibration
    UI.calibSlider.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        UI.calibValDisplay.textContent = val.toFixed(1);

        const lIdx = UI.calibLeg.value;
        const jIdx = UI.calibJoint.value;
        sendCommand(`CALIB:${lIdx}:${jIdx}:${val}`);
    });

    UI.btnSaveCalib.addEventListener('click', () => {
        sendCommand('SAVE');
    });

    // Built-in Poses
    UI.btnStand.addEventListener('click', () => {
        // Send neutral 90 to all joints
        for (let l = 1; l <= 4; l++) {
            sendCommand(`L${l}S1:90`);
            sendCommand(`L${l}S2:90`);
            sendCommand(`L${l}S3:90`);
            state.angles[l - 1] = { hip: 90, shoulder: 90, knee: 90 };
            update3DModel(l - 1);
        }
    });

    UI.btnSit.addEventListener('click', () => {
        for (let l = 1; l <= 4; l++) {
            // Adjust knees & shoulders to sit
            sendCommand(`L${l}S1:90`);
            sendCommand(`L${l}S2:130`);
            sendCommand(`L${l}S3:50`);
            state.angles[l - 1] = { hip: 90, shoulder: 130, knee: 50 };
            update3DModel(l - 1);
        }
    });

    UI.btnRest.addEventListener('click', () => {
        for (let l = 1; l <= 4; l++) {
            sendCommand(`L${l}S1:90`);
            sendCommand(`L${l}S2:180`);
            sendCommand(`L${l}S3:0`);
            state.angles[l - 1] = { hip: 90, shoulder: 180, knee: 0 };
            update3DModel(l - 1);
        }
    });

    // --- Directional Control Logic ---
    let walkInterval = null;
    let currentDir = null;

    function stopWalking() {
        if (walkInterval) clearInterval(walkInterval);
        walkInterval = null;
        currentDir = null;
        UI.btnFwd.classList.remove('active');
        UI.btnBack.classList.remove('active');
        UI.btnLeft.classList.remove('active');
        UI.btnRight.classList.remove('active');
        sendCommand('STOP');
        // Request the ESP32 to report its current state (which is now homeAngles)
        // so the UI sliders and 3D model re-sync to the saved pose
        setTimeout(() => sendCommand('REPORT'), 200);
    }

    function startWalking(dir, btnElement) {
        if (currentDir === dir) return;
        stopWalking();
        currentDir = dir;
        btnElement.classList.add('active');

        let speed = UI.sliderSpeed.value;
        // Tell firmware to start gait pattern and speed
        sendCommand(`WALK:${dir}:${speed}`);
    }

    UI.btnFwd.addEventListener('mousedown', () => startWalking('F', UI.btnFwd));
    UI.btnBack.addEventListener('mousedown', () => startWalking('B', UI.btnBack));
    UI.btnLeft.addEventListener('mousedown', () => startWalking('L', UI.btnLeft));
    UI.btnRight.addEventListener('mousedown', () => startWalking('R', UI.btnRight));
    UI.btnStop.addEventListener('mousedown', stopWalking);

    // Keyboard controls (WASD)
    window.addEventListener('keydown', (e) => {
        if (e.repeat) return; // ignore auto-repeat
        if (document.activeElement.tagName === "INPUT") return; // ignore if typing

        switch (e.key.toLowerCase()) {
            case 'w': startWalking('F', UI.btnFwd); break;
            case 's': startWalking('B', UI.btnBack); break;
            case 'a': startWalking('L', UI.btnLeft); break;
            case 'd': startWalking('R', UI.btnRight); break;
            case ' ': stopWalking(); break;
        }
    });

    window.addEventListener('keyup', (e) => {
        if (document.activeElement.tagName === "INPUT") return;
        const key = e.key.toLowerCase();
        if ((key === 'w' && currentDir === 'F') ||
            (key === 's' && currentDir === 'B') ||
            (key === 'a' && currentDir === 'L') ||
            (key === 'd' && currentDir === 'R')) {
            stopWalking();
        }
    });

    // Initialize viewers array 
    for (let i = 0; i < 4; i++) {
        update3DModel(i);
    }
});
