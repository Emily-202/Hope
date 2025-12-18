from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
import math
import urllib.parse, json
from urllib.request import urlopen
import multiprocessing
from shifter import Shifter
import time
from RPi import GPIO

## GPIO Setup ------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)
time.sleep(1)
laserpin=23
GPIO.setup(laserpin, GPIO.OUT)
GPIO.output(laserpin, GPIO.LOW)

## Global Variables ------------------------------------------------------------------
Globalradius=167.64
Globalangle=0
Globalheight=20.955

## Helpful Websites ------------------------------------------------------------------
# https://www.w3schools.com/css/css3_buttons.asp
# class server --> http://192.168.1.254:8000/positions.json
# test --> http://192.168.66.122:8000/positions.json


## Find JSON File --------------------------------------------------------------------

def load_target_data(url="http://192.168.66.122:8000/positions.json"):
    # Return parsed JSON (dict) from a URL using urllib
    try:
        with urlopen(url) as response:
            return json.load(response)
    except Exception as e:
        print("Error loading JSON from URL:", e)
        # Fallback: try loading local file
        try:
            with open("targets.json", "r") as f:
                return json.load(f)
        except Exception as e:
            print("Error loading local JSON:", e)
            return {}
        return {}


## Get Theta and Z Values from JSON --------------------------------------------------
def extract_theta_z(data_text):
    """
    Parse JSON text containing turrets and globes and return a dictionary
    with key-value pairs of theta and z (if available) for all items.
    """
    # Convert the JSON text to a Python dictionary
    #data = json.loads(data_text)

    result = {}

    # Extract theta for all turrets
    for turret_id, turret_data in data.get("turrets", {}).items():
        result[f"turret_{turret_id}_theta"] = turret_data.get("theta")

    # Extract theta and z for all globes
    for i, globe_data in enumerate(data.get("globes", []), start=1):
        result[f"globe_{i}_theta"] = globe_data.get("theta")
        result[f"globe_{i}_z"] = globe_data.get("z")

    return result

## Position Values -------------------------------------------------------------------
bedRotation = {'A':0}
laserRotation = {'B':0}
laserState = {"on": False}

## Generate HTML Code ----------------------------------------------------------------
def generateHTML():
    laser_color = "green" if laserState["on"] else "red"
    laser_text = "ON" if laserState["on"] else "OFF"

    html = f"""
    <html>
    <head>
        <title>Stepper Control</title>
        <meta charset="UTF-8">
        <style>
            #leftPanel {{
                width: 55%;
            }}
            #rightPanel {{
                width: 40%;
                border: 2px solid #333;
                border-radius: 10px;
                padding: 20px;
                background-color: #f0f0f0;
                height: fit-content;
            }}
            #orientationBox {{
                font-size: 18px;
                line-height: 1.6em;
            }}
            h3 {{
                margin-top: 10px;
            }}
            .fancyButton {{
                display: inline-block;
                padding: 15px 25px;
                font-size: 24px;
                cursor: pointer;
                text-align: center;
                text-decoration: none;
                color: #fff;
                background-color: #30ba6c;
                border: none;
                border-radius: 15px;
                width: 250px;
                box-shadow: 0 9px #999;
            }}
            .fancyButton:active {{
                box-shadow: 0 5px #666;
                transform: translateY(4px);
            }}
        </style>
    </head>
    <body style="font-family: Arial; margin: 30px;">

    <!-- LEFT SIDE (controls) -->
        <h3> Stepper Axis Control </h3>
        <p> Use the input fields below to set the desired positions for each axis. <br>
            Click the buttons to move the axes (in degrees) or zero their positions.</p>

            <div style="display: flex; flex-direction: row; gap: 40px; align-items: flex-start;">
            <div style="flex: 1; min-width: 350px;">

            <div>
                <p>
                    <label for="bedRotation">Bed Position [-80 and 80]:</label>
                    <input type="number" id="bedRotation" min="-80" max="80" value="{bedRotation['A']}"><br><br>
                </p>
                <p>
                    <label for="laserRotation">Laser Position [-80 and 80]:</label>
                    <input type="number" id="laserRotation" min="-80" max="80" value="{laserRotation['B']}"><br><br>
                </p>
                <input type="button" value="Move" onclick="moveMotors();">
                <input type="button" value="Zero Positions" onclick="zeroMotors();">
            </div>

            <br><hr><br>

            <h3>Laser Control</h3>
            <div id="laserIndicator"
                style="width:40px; height:40px; border-radius:50%; background:{laser_color};
                        display:inline-block; vertical-align:middle; margin-right:10px;"></div>
            <span id="laserStatus" style="font-weight:bold;">Laser is {laser_text}</span>
            <br><br>
            <input type="button" id="laserButton" value="Toggle Laser" onclick="toggleLaser();">

            <br><hr><br>

            <h3>Set Robot Position</h3>
            <select id="robotPosSelector">
                <option value="">-- Choose turret as robot position --</option>
            </select>
            <br><br>
            <input type="button" value="Set Robot Position" onclick="setRobotPosition();">

            <br><hr><br>

            <h3>Select Target</h3>
                <select id="targetSelector">
                    <option value="">-- Choose a target --</option>
                </select>
            <br><br>
            <input type="button" id="moveTargetButton" value="Move to Target" onclick="moveToTarget();">
        </div>

    <!-- RIGHT SIDE (orientation display) -->
        <div style="flex: 1; min-width: 350px;">
            <h2>Robot Orientation</h2>
            <div id="orientationBox">
                <p><b>Bed Rotation:</b> <span id="bedAngleDisplay">{bedRotation['A']}</span>°</p>
                <p><b>Laser Rotation:</b> <span id="laserAngleDisplay">{laserRotation['B']}</span>°</p>
            </div>
            <br><br><br><br>
            <input type="button" value="start" onclick="startTrial();" class="fancyButton">
        </div>

    <script>
        function updateOrientationDisplay() {{
            document.getElementById('bedAngleDisplay').textContent =
                document.getElementById('bedRotation').value;
            document.getElementById('laserAngleDisplay').textContent =
                document.getElementById('laserRotation').value;
        }}

        async function sendValue(axis, value, isZero=false) {{
            const body = new URLSearchParams();
            body.append(axis, value);
            if (isZero) body.append("zero", "true");

            const response = await fetch('/', {{
                method: 'POST',
                headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
                body: body
            }});
            try {{
                await response.json();
            }} catch (e) {{
                console.error("Non-JSON response");
            }}
            updateOrientationDisplay();
        }}

        function moveMotors() {{
            let bed = parseFloat(document.getElementById('bedRotation').value);
            let laser = parseFloat(document.getElementById('laserRotation').value);
            if (isNaN(bed) || bed < -90 || bed > 90) return alert("Bed value must be between -90 and 90.");
            if (isNaN(laser) || laser < -90 || laser > 90) return alert("Laser value must be between -90 and 90.");
            sendValue("bedRotation", bed);
            sendValue("laserRotation", laser);
        }}

        function zeroMotors() {{
            document.getElementById('bedRotation').value = 0;
            document.getElementById('laserRotation').value = 0;
            sendValue("bedRotation", 0, true);
            sendValue("laserRotation", 0, true);
            updateOrientationDisplay();
        }}

        async function toggleLaser() {{
            try {{
                const response = await fetch('/toggleLaser', {{ method: 'POST' }});
                const result = await response.json();

                const indicator = document.getElementById('laserIndicator');
                const status = document.getElementById('laserStatus');
                if (result.on) {{
                    indicator.style.background = 'green';
                    status.textContent = 'Laser is ON';
                }} else {{
                    indicator.style.background = 'red';
                    status.textContent = 'Laser is OFF';
                }}
            }} catch (err) {{
                console.error("Error toggling laser:", err);
                alert("Failed to toggle laser. See console for details.");
                return;
            }}
        }}

        /* ================= CONSTANTS ================= */
        const R = 167.64;          // cm
        const LASER_H = 20.955;
        const MIN = -80;
        const MAX = 80;

        /* ================= ROBOT STATE ================= */
        let robotTheta = 0;    // radians (UPDATED when robot position is set)

        /* ================= HELPERS ================= */
        function clamp(v,min,max){{ return Math.max(min,Math.min(max,v)); }}

        function shortestAngleRad(theta){{
            return (theta + Math.PI) % (2*Math.PI) - Math.PI;
        }}

        function chordDistance(dTheta){{
            return 2 * R * Math.sin(Math.abs(dTheta) / 2);
        }}

        function laserAngle(dTheta, targetH){{
            const D = Math.max(chordDistance(dTheta), 1e-6);
            return Math.atan2(targetH - LASER_H, D) * 180 / Math.PI;
        }}


        async function moveToTarget() {{
            const selected = document.getElementById('targetSelector').value;
            if (!selected) {{
                alert("Please select a target first.");
                return;
            }}

            // Current robot orientation from UI
            const bed = document.getElementById('bedRotation').value;
            const laser = document.getElementById('laserRotation').value;

            const body = new URLSearchParams();
            body.append("chosenTarget", selected);
            body.append("robotPosition", `${{bed}},${{laser}}`);

            let result;

            try {{
                const response = await fetch('/moveToTarget', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
                    body
                }});
                result = await response.json();
            }} catch (err) {{
                console.error("Move request failed:", err);
                alert("Failed to send move command.");
                return;
            }}
            if (!result.success) {{
                alert(result.message || "Move failed.");
                return;
            }}

            // document.getElementById('bedRotation').value = bedDeg;
            // document.getElementById('laserRotation').value = laserDeg;
            document.getElementById('bedRotation').value = result.bed.toFixed(1);
            document.getElementById('laserRotation').value = result.laser.toFixed(1);

            // await sendValue("bedRotation", bedDeg);
            // await sendValue("laserRotation", laserDeg);
            updateOrientationDisplay();

            // Laser ON (3 sec) then OFF
            await fetch('/toggleLaser', {{ method: 'POST' }});
            await new Promise(r => setTimeout(r, 3000));
            await fetch('/toggleLaser', {{ method: 'POST' }});
        }}



        // Store robot position in JS (absolute angles)
        let robotPosition = {{
            bed: null,
            laser: null
        }};

        // Code to populate drowpdown with turret/globe positions
        async function loadTargets() {{
            const resp = await fetch('/targets');
            const data = await resp.json();

            // ====== TARGET DROPDOWN ======
            const selector = document.getElementById('targetSelector');
            selector.innerHTML = "";
            const defaultOption = document.createElement('option');
            defaultOption.value = "";
            defaultOption.textContent = "-- Choose a target --";
            selector.appendChild(defaultOption);

            // Turrets
            if (data.turrets) {{
                const groupTurrets = document.createElement('optgroup');
                groupTurrets.label = "Turrets";

                for (const [id, vals] of Object.entries(data.turrets)) {{
                    const option = document.createElement('option');
                    option.value = `turret_${{id}}`;
                    option.textContent = `Turret ${{id}} → θ=${{vals.theta.toFixed(3)}} rad`;
                    groupTurrets.appendChild(option);
                }}
                selector.appendChild(groupTurrets);
            }}

            // Globes
            if (data.globes) {{
                const groupGlobes = document.createElement('optgroup');
                groupGlobes.label = "Globes";

                data.globes.forEach((g, i) => {{
                    const option = document.createElement('option');
                    option.value = `globe_${{i+1}}`;
                    option.textContent = `Globe ${{i+1}} → θ=${{g.theta.toFixed(3)}} rad, z=${{g.z.toFixed(1)}}`;
                    groupGlobes.appendChild(option);
                }});
                selector.appendChild(groupGlobes);
            }}

            // Set the robot position dropdown list
            const robSel = document.getElementById('robotPosSelector');
            robSel.innerHTML = "";
            const defaultRobot = document.createElement('option');
            defaultRobot.value = "";
            defaultRobot.textContent = "-- Choose turret as robot position --";
            robSel.appendChild(defaultRobot);

            for (const [id, vals] of Object.entries(data.turrets || {{}})) {{
                const option = document.createElement('option');
                option.value = `turret_${{id}}`;
                option.textContent = `Turret ${{id}} (θ=${{vals.theta.toFixed(3)}} rad)`;
                robSel.appendChild(option);
            }}
        }}


        // Set the robot position
        async function setRobotPosition() {{
            const sel = document.getElementById('robotPosSelector');
            const choice = sel.value;
            if (!choice) return alert("Select a turret position first.");

            const id = choice.split("_")[1];

            // Load JSON so we know the turret positions
            const data = await (await fetch('/targets')).json();
            const turret = data.turrets[id];

            if (!turret) return alert("Invalid turret selected.");

            // Convert to degrees (absolute)
            const bedDeg = turret.theta * 180 / Math.PI;
            const bedRad = turret.theta;
            const laserDeg = 0;  // Always zero when robot is at a turret

            try {{
                let response = await fetch('/setRobotPosition', {{
                    method: "POST",
                    headers: {{ "Content-Type": "application/x-www-form-urlencoded" }},
                    body: `bed=${{bedRad}}&laser=${{laserDeg}}`
                }});

                const result = await response.json();
                console.log("[SERVER RESPONSE]", result);
            }} catch (err) {{
                console.error("Error sending robot position:", err);
            }}

            alert(`Robot position set to Turret ${{id}}.\nBed=${{bedDeg.toFixed(1)}}°, Laser=0°`);
        }}
        
        async function startTrial() {{
            const data = await (await fetch('/targets')).json();

            // Build ordered target list (strings match backend expectations)
            let targets = [];

            // Turrets first
            for (const id of Object.keys(data.turrets || {{}})) {{
                targets.push(`turret_${{id}}`);
            }}

            // Then globes
            (data.globes || []).forEach((_, i) => {{
                targets.push(`globe_${{i + 1}}`);
            }});

            if (targets.length === 0) {{
                alert("No targets found.");
                return;
            }}

            // Current robot orientation from UI
            let bed = document.getElementById('bedRotation').value;
            let laser = document.getElementById('laserRotation').value;

            for (let i = 0; i < targets.length; i++) {{
                const target = targets[i];
                console.log(`▶ Target ${{i + 1}}/${{targets.length}}: ${{target}}`);

                const body = new URLSearchParams();
                body.append("chosenTarget", target);
                body.append("robotPosition", `${{bed}},${{laser}}`);

                let result;
                try {{
                    const response = await fetch('/moveToTarget', {{
                        method: 'POST',
                        headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
                        body
                    }});
                    result = await response.json();
                }} catch (err) {{
                    console.error("Move failed:", err);
                    alert("Autonomous trial aborted.");
                    return;
                }}

                if (!result.success) {{
                    alert(result.message || "Move failed.");
                    return;
                }}

                // Update UI with ACTUAL motor angles returned by backend
                bed = result.bed;
                laser = result.laser;

                document.getElementById('bedRotation').value = bed.toFixed(1);
                document.getElementById('laserRotation').value = laser.toFixed(1);
                updateOrientationDisplay();

                // Laser ON (3 sec)
                await fetch('/toggleLaser', {{ method: 'POST' }});
                await new Promise(r => setTimeout(r, 3000));
                await fetch('/toggleLaser', {{ method: 'POST' }});

                // Small pause before next target
                await new Promise(r => setTimeout(r, 500));
            }}

            alert("Autonomous trial complete.");
        }}


        loadTargets();
        updateOrientationDisplay();
    </script>

    </body>
    </html>
    """
    return html.encode("utf-8")



## Run Server Command ----------------------------------------------------------------
def runServer():
    server_address = ("0.0.0.0", 8080)
    httpd = ThreadingHTTPServer(server_address, StepperHandler)
    print("Server running on http://<pi-ip>:8080/ (Press Ctrl+C to stop)")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        httpd.server_close()
        print("Server stopped cleanly.")
        GPIO.cleanup()


## Extra Functions -------------------------------------------------------------------


## HTTP Request Handler --------------------------------------------------------------
class StepperHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(generateHTML())
        elif self.path == '/targets':
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            targets = load_target_data()
            self.wfile.write(json.dumps(targets).encode('utf-8'))
        else:
            self.send_error(404)

    def do_POST(self):

        # Setting turret position update
        if self.path == "/setRobotPosition":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length).decode("utf-8")
            parsed = urllib.parse.parse_qs(body)

            global Globalangle, Globalradius

            try:
                Globalangle = float(parsed.get("bed", [0])[0])
                Globalradius = 167.64
                print(f"Robot position set: angle={Globalangle}, radius={Globalradius}")
            except:
                print("Invalid robot position POST")

            self._send_json({"success": True})
            return

        # Laser toggle update
        if self.path == "/toggleLaser":
            # Flip the state
            laserState["on"] = not laserState["on"]
            print(f"Laser toggled {'ON' if laserState['on'] else 'OFF'}")

            # Actually turn the laser ON or OFF
            if laserState["on"]:
                GPIO.output(laserpin, GPIO.HIGH)
            else:
                GPIO.output(laserpin, GPIO.LOW)

            # Send response
            self._send_json({"success": True, "on": laserState["on"]})
            return

        # Target/Globe selection update
        if self.path == "/selectTarget":
            # read posted target name
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8')
            parsed = urllib.parse.parse_qs(body)
            target_name = parsed.get('target', [''])[0]

            data = load_target_data()
            if target_name.startswith('turret_'):
                tid = target_name.split("_")[1]
                turret = data.get("turrets", {}).get(tid)
                if not turret:
                    self._send_json({"success": False, "message": "Turret not found"})
                    return
                target_theta = turret["theta"]


                bed_angle_deg = math.degrees(target_theta)
                laser_angle_deg = math.degrees(
                    math.atan2(-Globalheight,
                            2 * Globalradius * math.sin((target_theta - Globalangle) / 2))
                )

                self._send_json({
                    "success": True,
                    "bed": bed_angle_deg,
                    "laser": laser_angle_deg
                })
                return

            elif target_name.startswith('globe_'):
                gid = int(target_name.split('_')[1]) - 1
                try:
                    g = data.get('globes', [])[gid]
                    msg = f"Globe {gid+1}: r={g.get('r')}, theta={g.get('theta')}, z={g.get('z')}"
                except Exception:
                    msg = "Globe not found."
            else:
                msg = "Unknown target."

            print("Selected target:", msg)
            # reply with plain text (client reads it)
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(msg.encode('utf-8'))
            return

        if self.path == "/moveToTarget":
            # Read POST data
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length).decode("utf-8")
            parsed = urllib.parse.parse_qs(body)

            target_name = parsed.get("chosenTarget", [""])[0]
            robot_pos_str = parsed.get("robotPosition", [""])[0]

            print(f"[MOVE TO TARGET] robot at: {robot_pos_str}, target selected: {target_name}")

            # Look at robot current position
            try:
                robot_bed_deg, robot_laser_deg = map(float, robot_pos_str.split(","))
            except Exception:
                robot_bed_deg = robot_laser_deg = 0  # default if not provided

            # Load the JSON target data
            data = load_target_data()

            bed_angle_deg = 0
            laser_angle_deg = 0

            # Determine target type and compute required angles
            if target_name.startswith("turret_"):
                tid = target_name.split("_")[1]
                turret = data.get("turrets", {}).get(tid)
                if turret:
                    target_theta = turret["theta"]
                    target_z = 0.5  # Always aiming at base of turrets

                    # 🚫 SKIP if turret is at robot's current angular position
                    ANG_EPS = math.radians(2.0)
                    if abs(target_theta - Globalangle) < ANG_EPS:
                        print("[SKIP] Turret is at robot angular position")
                        self._send_json({
                            "success": False,
                            "message": "Target skipped (same angular position as robot)"
                        })
                        return

                    laser_angle_deg = self.motor_laser.goAngleY(target_theta, target_z)
                    bed_angle_deg = self.motor_bed.goAngleXZ(target_theta)

                else:
                    self._send_json({"success": False, "message": "Turret not found"})
                    return

            elif target_name.startswith("globe_"):
                gid = int(target_name.split("_")[1]) - 1
                globes = data.get("globes", [])
                if gid < 0 or gid >= len(globes):
                    self._send_json({"success": False, "message": "Globe not found"})
                    return

                globe = globes[gid]

                # --- Use Stepper movement system formulas ---
                # Bed angle: move in XZ plane (2D angular displacement)
                target_theta_rad = globe["theta"]

                # Laser angle: move in Y plane (height difference)
                target_z = globe.get("z", 0)
                bed_angle_deg = self.motor_bed.goAngleXZ(target_theta_rad)
                laser_angle_deg = self.motor_laser.goAngleY(target_theta_rad, target_z)

            else:
                self._send_json({"success": False, "message": "Unknown target"})
                return

            print(f"Commanding bed → {bed_angle_deg:.1f}°, laser → {laser_angle_deg:.1f}°")

            # return JSON response with final angles
            bed_angle_deg = max(-80, min(80, bed_angle_deg))
            laser_angle_deg = max(-80, min(80, laser_angle_deg))
            self._send_json({
                "success": True,
                "bed": bed_angle_deg,
                "laser": laser_angle_deg
            })
            return

        # otherwise handle normal axis control as before
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length).decode()
        params = urllib.parse.parse_qs(body)


        print("Received POST data:", params)
        is_zero = "zero" in params

        for key in params:
            if key == "zero":
                continue  # skip the flag itself

            try:
                value = float(params[key][0])
            except:
                self._send_json({"success": False, "message": "Invalid number format"})
                return

            # Validate input range
            if value < -180 or value > 180:
                self._send_json({"success": False, "message": f"{key} must be between -180 and 180"})
                return

            # Save and call motor functions (only if not zeroing)
            if key == "bedRotation":
                bedRotation['A'] = value
                if not is_zero:
                    try:
                        self.motor_bed.goAngle(float(value))
                        print(f"[BED] commanded to {value}°")
                    except Exception as e:
                        print("Error moving bed motor:", e)
                else:
                    # Proper zeroing
                    try:
                        self.motor_bed.zero()
                        print("Bed axis zeroed.")
                    except Exception as e:
                        print("Error zeroing bed motor:", e)

            elif key == "laserRotation":
                laserRotation['B'] = value
                if not is_zero:
                    try:
                        self.motor_laser.goAngle(float(value))
                        print(f"[LASER] commanded to {value}°")
                    except Exception as e:
                        print("Error moving laser motor:", e)
                else:
                    # Proper zeroing
                    try:
                        self.motor_laser.zero()
                        print("Laser axis zeroed.")
                    except Exception as e:
                        print("Error zeroing laser motor:", e)

        self._send_json({"success": True})


    # JSON response helper
    def _send_json(self, obj):
        response = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(response)


## Stepper Class ---------------------------------------------------------------------
class Stepper:
    # Class attributes
    num_steppers = 0      # track number of Steppers instantiated
    shifter_outputs = 0   # track shift register outputs for all motors
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 2500          # delay between motor steps [us]
    steps_per_degree = 4096/360     # 4096 steps/rev * 1/360 rev/deg

    def __init__(self, shifter, lock):
        self.s = shifter            # shift register
        self.angle = multiprocessing.Value('d', 0.0)  # current output shaft angle
        self.step_state = 0         # track position in sequence
        self.shifter_bit_start = 4*Stepper.num_steppers  # starting bit position
        self.lock = lock            # multiprocessing lock

        Stepper.num_steppers += 1   # increment the instance count

    # Signum function:
    def __sgn(self, x):
        if x == 0: return(0)
        else: return(int(abs(x)/x))

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        self.step_state += dir    # increment/decrement the step
        self.step_state %= 8      # ensure result stays in [0,7]

        # 4-bit coil pattern for this motor
        mask   = 0b1111 << self.shifter_bit_start
        pattern = Stepper.seq[self.step_state] << self.shifter_bit_start

        # Clear existing bits only for this motor
        Stepper.shifter_outputs &= ~mask
        # Set this motor's new coil pattern
        Stepper.shifter_outputs |= pattern

        self.s.shiftByte(Stepper.shifter_outputs)

        # update shared angle
        with self.angle.get_lock():
            self.angle.value += dir / Stepper.steps_per_degree
            self.angle.value %= 360

    # Move relative angle from current position:
    def __rotate(self, delta):
        self.lock.acquire()                 # wait until the lock is available
        numSteps = int(Stepper.steps_per_degree * abs(delta))    # find the right # of steps
        dir = self.__sgn(delta)        # find the direction (+/-1)
        for s in range(numSteps):      # take the steps
            self.__step(dir)
            time.sleep(Stepper.delay/1e6)
        self.lock.release()

    # Move relative angle from current position:
    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, tarAngle):
        # read angle safely

        with self.angle.get_lock():
            curAngle = self.angle.value

        # shortest path math: force into [-180, 180]
        if (tarAngle>80):
            tarAngle=80
        elif (tarAngle<-80):
            tarAngle=-80
        delta = ((tarAngle - curAngle + 540) % 360) - 180
        #delta = tarAngle - curAngle    

        print(f'delta: {delta}')
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()
        p.join()

    # moves the motor in the XZ when given our angular position with respect to the center
    # and zero and a targets angular position with respect to the center 
    def goAngleXZ(self, targetAngle):
        alpha=.5*(math.pi-abs(targetAngle-Globalangle))
        alpha=math.degrees(alpha)
        if (targetAngle-Globalangle >0):
            alpha=-alpha
        self.goAngle(alpha)
        return alpha

    # moves the motor in the Y when given our angular position with respect to the center
    # and zero and a targets angular position with respect to the center
    # and zero and circle radius our own height and target height     
    def goAngleY(self, targetAngle,targetHeight):
        # Signed angular difference around circle (radians)
        dtheta = targetAngle - Globalangle
        dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi

        # Horizontal distance along ring (ARC length)
        C = Globalradius * abs(dtheta)

        # Prevent divide-by-zero
        if C < 1e-6:
            print("[AngleY] Target inline — skipping tilt")
            return

        # Vertical difference (cm)
        # Globalheight must be the LASER HEIGHT (20.955 cm)
        dh = targetHeight - Globalheight

        # Tilt angle (negative = down)
        phi = math.atan2(dh, C)
        phi_deg = math.degrees(phi)

        # Clamp to mechanical limits
        phi_deg = max(-80.0, min(80.0, phi_deg))

        print(
            f"[AngleY] Δθ={math.degrees(dtheta):.1f}°, "
            f"C={C:.1f}cm, "
            f"Δh={dh:.3f}cm, "
            f"φ={phi_deg:.2f}°"
        )

        self.goAngle(phi_deg)
        return phi_deg

    def hoizontalZero(self):
        theta=math.atan2(Globalheight,Globalradius)
        theta=math.degrees(theta)
        p = multiprocessing.Process(target=self.__rotate, args=(theta,))
        p.start()
        p.join()

    # Set the motor zero point
    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


## Run Code --------------------------------------------------------------------------
if __name__ == "__main__":
    s = Shifter(data=14,latch=15,clock=18)   # set up Shifter

    # Use multiprocessing.Lock() to prevent motors from trying to 
    # execute multiple operations at the same time:
    lock1 = multiprocessing.Lock()
    #lock2 = multiprocessing.Lock()

    # Instantiate 2 Steppers:
    m1 = Stepper(s, lock1)
    m2 = Stepper(s, lock1)

    # Zero the motors:
    m1.zero()
    m2.zero()

    # Attach to handler so handler can move motors
    StepperHandler.motor_laser = m2
    StepperHandler.motor_bed = m1

    try: 
        runServer()
    except Exception as e:
        print("Error running server:", e)
        GPIO.cleanup()