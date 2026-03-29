class RobotViewer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        
        // Create a specific div for Plotly to render into, inserted behind overlays
        this.plotDiv = document.createElement('div');
        this.plotDiv.style.width = '100%';
        this.plotDiv.style.height = '100%';
        this.plotDiv.style.position = 'absolute';
        this.plotDiv.style.top = '0';
        this.plotDiv.style.left = '0';
        this.plotDiv.style.zIndex = '0';
        this.container.insertBefore(this.plotDiv, this.container.firstChild);

        // State to hold the latest FK positions of each leg (4 arrays of 4 points)
        this.legJoints = [ [], [], [], [] ];
        
        // Colors matching the webapp's Neon Cyan theme
        this.colors = {
            bodyMesh: '#00f0ff',       // Cyan body fill
            bodyOutline: '#00c3cf',    // Slightly darker cyan outline
            headMarker: '#ffffff',     // White head marker
            cogMarker: '#2ea043',      // Success green from theme
            legLines: '#00f0ff',       // Cyan legs
            legJoints: '#ffffff',      // White leg joints
            supportPolygon: '#00f0ff', // Cyan translucent support
            gridLines: 'rgba(0, 240, 255, 0.3)', // Faint cyan grid
            ground: 'rgba(20, 20, 28, 0.8)',     // Dark glass panel background
            axisX: '#ff3366',          // Theme danger red
            axisY: '#2ea043',          // Theme success green
            axisZ: '#00f0ff'           // Theme accent cyan
        };

        this.initPlot();

        // Handle resizing
        window.addEventListener('resize', () => {
            Plotly.Plots.resize(this.plotDiv);
        });
    }

    initPlot() {
        const bl = Kinematics.DIM.BODY_L / 2;
        const bw = Kinematics.DIM.BODY_W / 2;

        // Body Mesh & Outline vertices (FL, FR, BR, BL, FL)
        const mountX = [bl, bl, -bl, -bl, bl];
        const mountY = [bw, -bw, -bw, bw, bw];
        const mountZ = [0, 0, 0, 0, 0];

        // Head marker
        const headX = [bl];
        const headY = [0];
        const headZ = [0];

        // Pre-compute legs at neutral position (90, 90, 90)
        for (let i = 0; i < 4; i++) {
            this.legJoints[i] = Kinematics.solveFK(i, 90, 90, 90);
        }

        // Ordered for support polygon perimeter: FL(0), FR(1), BR(3), BL(2)
        const polyOrder = [0, 1, 3, 2];
        const polyX = polyOrder.map(i => this.legJoints[i][3].x);
        const polyY = polyOrder.map(i => this.legJoints[i][3].y);
        const polyZ = polyOrder.map(i => this.legJoints[i][3].z);

        const traces = [
            // [0] Body Mesh Translucent
            {
                name: 'bodyMesh',
                type: 'mesh3d',
                opacity: 0.3,
                color: this.colors.bodyMesh,
                x: mountX.slice(0, 4), 
                y: mountY.slice(0, 4), 
                z: mountZ.slice(0, 4),
                i: [0, 0],
                j: [1, 2],
                k: [2, 3],
                hoverinfo: 'none'
            },
            // [1] Body Outline Wireframe
            {
                name: 'bodyOutline',
                type: 'scatter3d',
                mode: 'lines',
                line: { color: this.colors.bodyOutline, width: 12 },
                x: mountX, 
                y: mountY, 
                z: mountZ,
                hoverinfo: 'none'
            },
            // [2] Head Marker
            {
                name: 'headMarker',
                type: 'scatter3d',
                mode: 'markers',
                marker: { color: this.colors.headMarker, size: 14, symbol: 'circle' },
                x: headX, 
                y: headY, 
                z: headZ,
                hoverinfo: 'none'
            },
            // [3] COG Marker
            {
                name: 'cogMarker',
                type: 'scatter3d',
                mode: 'markers',
                marker: { color: this.colors.cogMarker, size: 14, symbol: 'circle' },
                x: [0], y: [0], z: [0],
                hoverinfo: 'none'
            },
            // [4..7] Legs
            ...Array.from({length: 4}).map((_, i) => {
                const legNames = ['Leg 1 (FL)', 'Leg 2 (FR)', 'Leg 3 (BL)', 'Leg 4 (BR)'];
                const jointNames = ['Hip Mount', 'Shoulder Joint', 'Knee Joint', 'Foot Tip'];
                return {
                    name: `leg${i}`,
                    type: 'scatter3d',
                    mode: 'lines+markers',
                    line: { color: this.colors.legLines, width: 10 },
                    marker: { color: this.colors.legJoints, size: 6, symbol: 'circle' },
                    x: this.legJoints[i].map(p => p.x),
                    y: this.legJoints[i].map(p => p.y),
                    z: this.legJoints[i].map(p => p.z),
                    text: jointNames.map(j => `${legNames[i]} - ${j}`),
                    hoverinfo: 'none' // Handled by custom HTML
                };
            }),
            // [8] Support Polygon Mesh
            {
                name: 'supportPolygon',
                type: 'mesh3d',
                opacity: 0.2, // Matches hexapod-master
                color: this.colors.supportPolygon,
                x: polyX,
                y: polyY,
                z: polyZ,
                i: [0, 0],
                j: [1, 2],
                k: [2, 3],
                hoverinfo: 'none'
            },
            // [9] X Axis (Forward/Red)
            {
                name: 'xAxis',
                type: 'scatter3d',
                mode: 'lines',
                line: { color: this.colors.axisX, width: 3 },
                x: [0, 150], y: [0, 0], z: [0, 0],
                hoverinfo: 'none'
            },
            // [10] Y Axis (Left/Green)
            {
                name: 'yAxis',
                type: 'scatter3d',
                mode: 'lines',
                line: { color: this.colors.axisY, width: 3 },
                x: [0, 0], y: [0, 150], z: [0, 0],
                hoverinfo: 'none'
            },
            // [11] Z Axis (Up/Blue)
            {
                name: 'zAxis',
                type: 'scatter3d',
                mode: 'lines',
                line: { color: this.colors.axisZ, width: 3 },
                x: [0, 0], y: [0, 0], z: [0, 150],
                hoverinfo: 'none'
            }
        ];

        const layout = {
            margin: { l: 0, r: 0, b: 0, t: 0 },
            paper_bgcolor: 'rgba(0,0,0,0)',
            plot_bgcolor: 'rgba(0,0,0,0)',
            scene: {
                xaxis: { 
                    showgrid: false, zeroline: true, showticklabels: false, title: '', showbackground: false,
                    zerolinecolor: this.colors.gridLines, zerolinewidth: 2
                },
                yaxis: { 
                    showgrid: false, zeroline: true, showticklabels: false, title: '', showbackground: false,
                    zerolinecolor: this.colors.gridLines, zerolinewidth: 2
                },
                zaxis: { 
                    showgrid: false, zeroline: true, showticklabels: false, title: '',
                    showbackground: true, backgroundcolor: this.colors.ground,
                    zerolinecolor: this.colors.gridLines, zerolinewidth: 2,
                    range: [-180, 70]
                },
                aspectmode: 'manual',
                aspectratio: { x: 1, y: 1, z: 0.5 },
                camera: {
                    eye: { x: -1.7, y: -1.7, z: 1.0 }
                }
            },
            showlegend: false
        };

        const config = {
            displaylogo: false,
            responsive: true,
            displayModeBar: true,
            modeBarButtonsToRemove: ['lasso2d', 'select2d']
        };

        Plotly.newPlot(this.plotDiv, traces, layout, config);

        // Setup Custom HTML Tooltip logic
        this.setupCustomTooltip();
    }

    setupCustomTooltip() {
        this.hoverTooltip = document.createElement('div');
        this.hoverTooltip.className = 'custom-tooltip';
        this.hoverTooltip.style.opacity = '0';
        document.body.appendChild(this.hoverTooltip);

        // Keep track of cursor globally for tooltips on 3D objects
        this.mouseX = 0;
        this.mouseY = 0;
        document.addEventListener('mousemove', (e) => {
            this.mouseX = e.clientX;
            this.mouseY = e.clientY;
            
            // Allow tooltip to stick to the mouse if it's visible
            if (this.hoverTooltip.style.opacity === '1') {
                this.hoverTooltip.style.left = (this.mouseX + 15) + 'px';
                this.hoverTooltip.style.top = (this.mouseY + 15) + 'px';
            }
        });

        this.plotDiv.on('plotly_hover', (data) => {
            if (data.points && data.points.length > 0) {
                const pt = data.points[0];
                if (!pt.text) return; // Skip if it's not a joint marker

                // Build the inner HTML identical to the button layout requests
                this.hoverTooltip.innerHTML = `
                    <div class="tooltip-title">${pt.text}</div>
                    <div class="tooltip-data">X: ${pt.x.toFixed(1)}</div>
                    <div class="tooltip-data">Y: ${pt.y.toFixed(1)}</div>
                    <div class="tooltip-data">Z: ${pt.z.toFixed(1)}</div>
                `;

                this.hoverTooltip.style.left = (this.mouseX + 15) + 'px';
                this.hoverTooltip.style.top = (this.mouseY + 15) + 'px';
                this.hoverTooltip.style.opacity = '1';
            }
        });

        this.plotDiv.on('plotly_unhover', () => {
             this.hoverTooltip.style.opacity = '0';
        });

        this.plotDiv.addEventListener('mouseleave', () => {
             this.hoverTooltip.style.opacity = '0';
        });
    }


    /**
     * Updates the visuals of a specific leg
     */
    updateLegJoints(legIndex, hipAngle, shoulderAngle, kneeAngle) {
        if (legIndex < 0 || legIndex > 3) return;

        // Fetch physical x, y, z points by using Forward Kinematics
        const joints = Kinematics.solveFK(legIndex, hipAngle, shoulderAngle, kneeAngle);
        this.legJoints[legIndex] = joints;

        // Separate points into trace arrays
        const xs = joints.map(p => p.x);
        const ys = joints.map(p => p.y);
        const zs = joints.map(p => p.z);

        // Update the corresponding leg trace (indexes 4, 5, 6, 7)
        const traceIndex = 4 + legIndex;
        Plotly.restyle(this.plotDiv, {
            x: [xs],
            y: [ys],
            z: [zs]
        }, [traceIndex]);

        // Dynamically update the Support Polygon Mesh (index 8) based on newest 4 leg tips
        const polyOrder = [0, 1, 3, 2];
        const polyX = polyOrder.map(i => this.legJoints[i][3].x);
        const polyY = polyOrder.map(i => this.legJoints[i][3].y);
        const polyZ = polyOrder.map(i => this.legJoints[i][3].z);

        Plotly.restyle(this.plotDiv, {
            x: [polyX],
            y: [polyY],
            z: [polyZ]
        }, [8]);
    }

    /**
     * Override camera to initial view
     */
    resetView() {
        Plotly.relayout(this.plotDiv, {
            'scene.camera': {
                eye: { x: -1.7, y: -1.7, z: 1.0 },
                center: { x: 0, y: 0, z: 0 },
                up: { x: 0, y: 0, z: 1 }
            }
        });
    }
}
