/**
 * webservice.h
 *
 * Minimal Webservice um eine Steuerung auf ein Smartphone zu bringen, mit der der Roboter gesteuert werden kann.
 * Signale werden dabei per Socket direkt übertragen, sobald sich eine Änderung ergibt.
 *
 * Autor: Claude.ai & Jeanette Müller
 * Datum: 2025
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// Access Point Einstellungen
const char *ssid = "RobotController";
const char *password = "robot1234"; // Mindestens 8 Zeichen

// Server Instanzen
WebServer server(80);
WebSocketsServer webSocket(81);

// Roboter Steuerungswerte
struct RobotControl
{
    float joystickX = 0;    // -100 bis 100
    float joystickY = 0;    // -100 bis 100
    float roll = 0;         // -10 bis 10
    float pitch = 0;        // -10 bis 10
    float yaw = 0;          // -10 bis 10
    float height = 1750;    // 30 bis 265
    float legextend = 1200; // 60 bis 200
} robotControl;

// HTML Seite
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Robot Controller</title>
    <style>
        * {
            box-sizing: border-box;
        }
        
        html, body {
            touch-action: manipulation;
            overscroll-behavior: none;
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #000;
            color: #fff;
            margin: 0;
            padding: 16px;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            overflow: hidden;
            position: fixed;
            width: 100%;
        }
        
        h1 {
            margin: 0 0 8px 0;
            font-size: 1.5rem;
            color: #00d4ff;
            text-shadow: 0 0 10px rgba(0, 212, 255, 0.5);
        }
        
        .status {
            font-size: 0.85rem;
            margin-bottom: 16px;
            padding: 6px 16px;
            border-radius: 20px;
            background: rgba(255,255,255,0.1);
        }
        
        .status.connected { background: rgba(0, 255, 136, 0.2); color: #00ff88; }
        .status.disconnected { background: rgba(255, 68, 68, 0.2); color: #ff4444; }
        
        .joystick-container {
            width: 300px;
            height: 300px;
            background: radial-gradient(circle, #2a2a4a 0%, #1a1a2e 100%);
            border-radius: 50%;
            position: relative;
            margin-bottom: 24px;
            box-shadow: inset 0 0 30px rgba(0,0,0,0.5), 0 0 20px rgba(0, 212, 255, 0.2);
            border: 2px solid rgba(0, 212, 255, 0.3);
            touch-action: none;
        }
        
        .joystick-knob {
            width: 70px;
            height: 70px;
            background: radial-gradient(circle at 30% 30%, #00d4ff, #0077aa);
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: pointer;
            box-shadow: 0 4px 15px rgba(0, 212, 255, 0.4);
            transition: box-shadow 0.2s;
        }
        
        .joystick-knob:active {
            box-shadow: 0 4px 25px rgba(0, 212, 255, 0.8);
        }
        
        .joystick-knob.dragging {
            transform: none !important;
        }
        
        .controls {
            width: 100%;
            max-width: 350px;
            display: flex;
            flex-direction: column;
            gap: 16px;
            background: #000;
        }
        
        .slider-group {
            background: rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 12px 16px;
        }
        
        .slider-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
        }
        
        .slider-label {
            font-weight: 600;
            font-size: 0.9rem;
            color: #00d4ff;
        }
        
        .slider-value {
            font-family: 'Courier New', monospace;
            font-size: 1.1rem;
            font-weight: bold;
            min-width: 50px;
            text-align: right;
        }
        
        input[type="range"] {
            width: 100%;
            height: 8px;
            -webkit-appearance: none;
            background: linear-gradient(to right, #ff4444, #444 50%, #00ff88);
            border-radius: 4px;
            outline: none;
        }
        
        input[type="range"].height-slider {
            background: linear-gradient(to right, #ff8800, #00d4ff);
        }
        
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 24px;
            height: 24px;
            background: #fff;
            border-radius: 50%;
            cursor: pointer;
            box-shadow: 0 2px 8px rgba(0,0,0,0.3);
        }
        
        .values-display {
            margin-top: 20px;
            padding: 12px;
            background: rgba(0,0,0,0.3);
            border-radius: 8px;
            font-family: 'Courier New', monospace;
            font-size: 0.75rem;
            color: #888;
        }
    </style>
</head>
<body>
    <h1>🤖 Robot Controller</h1>
    <div class="status disconnected" id="status">Verbinde...</div>
    
    <div class="joystick-container" id="joystickArea">
        <div class="joystick-knob" id="joystickKnob"></div>
    </div>
    
    <div class="controls">
    <div class="slider-group">
            <div class="slider-header">
                <span class="slider-label">HÖHE</span>
                <span class="slider-value" id="heightValue">1750</span>
            </div>
            <input type="range" class="height-slider" id="height" min="0" max="2650" value="1750">
        </div>

        <div class="slider-group">
            <div class="slider-header">
                <span class="slider-label">BEIN ÜBERHANG</span>
                <span class="slider-value" id="legextendValue">1200</span>
            </div>
            <input type="range" class="height-slider" id="legextend" min="600" max="2000" value="1200">
        </div>

        <div class="slider-group">
            <div class="slider-header">
                <span class="slider-label">ROLL</span>
                <span class="slider-value" id="rollValue">0</span>
            </div>
            <input type="range" id="roll" min="-100" max="100" value="0">
        </div>
        
        <div class="slider-group">
            <div class="slider-header">
                <span class="slider-label">PITCH</span>
                <span class="slider-value" id="pitchValue">0</span>
            </div>
            <input type="range" id="pitch" min="-100" max="100" value="0">
        </div>
        
        <div class="slider-group">
            <div class="slider-header">
                <span class="slider-label">YAW</span>
                <span class="slider-value" id="yawValue">0</span>
            </div>
            <input type="range" id="yaw" min="-100" max="100" value="0">
        </div>
        
        
    </div>

    <script>
        let ws;
        let joystickData = { x: 0, y: 0 };
        let isDragging = false;
        
        const joystickArea = document.getElementById('joystickArea');
        const joystickKnob = document.getElementById('joystickKnob');
        const statusEl = document.getElementById('status');
        
        // WebSocket Verbindung
        function connect() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81/');
            
            ws.onopen = () => {
                statusEl.textContent = '✓ Verbunden';
                statusEl.className = 'status connected';
                sendData();
            };
            
            ws.onclose = () => {
                statusEl.textContent = '✗ Getrennt';
                statusEl.className = 'status disconnected';
                setTimeout(connect, 2000);
            };
            
            ws.onerror = () => {
                ws.close();
            };
        }
        
        // Daten senden
        function sendData() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const data = {
                    jx: joystickData.x,
                    jy: joystickData.y,
                    roll: parseInt(document.getElementById('roll').value),
                    pitch: parseInt(document.getElementById('pitch').value),
                    yaw: parseInt(document.getElementById('yaw').value),
                    height: parseInt(document.getElementById('height').value),
                    legextend: parseInt(document.getElementById('legextend').value)
                };
                ws.send(JSON.stringify(data));
            }
        }
        
        // Joystick Logik
        function handleJoystick(e) {
            if (!isDragging) return;
            
            e.preventDefault();
            const rect = joystickArea.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            const knobRadius = 35;
            
            let clientX, clientY;
            if (e.touches) {
                clientX = e.touches[0].clientX;
                clientY = e.touches[0].clientY;
            } else {
                clientX = e.clientX;
                clientY = e.clientY;
            }
            
            let x = clientX - rect.left - centerX;
            let y = clientY - rect.top - centerY;
            
            const maxRadius = centerX - knobRadius;
            const distance = Math.sqrt(x * x + y * y);
            
            if (distance > maxRadius) {
                x = (x / distance) * maxRadius;
                y = (y / distance) * maxRadius;
            }
            
            joystickKnob.style.left = (centerX + x - knobRadius) + 'px';
            joystickKnob.style.top = (centerY + y - knobRadius) + 'px';
            
            joystickData.x = Math.round((x / maxRadius) * 100);
            joystickData.y = Math.round((-y / maxRadius) * 100);
            
            sendData();
        }
        
        function resetJoystick() {
            isDragging = false;
            joystickKnob.classList.remove('dragging');
            joystickKnob.style.left = '50%';
            joystickKnob.style.top = '50%';
            joystickKnob.style.transform = 'translate(-50%, -50%)';
            joystickData = { x: 0, y: 0 };
            sendData();
        }
        
        // Event Listener
        joystickArea.addEventListener('mousedown', (e) => {
            isDragging = true;
            joystickKnob.classList.add('dragging');
            joystickKnob.style.transform = 'none';
            handleJoystick(e);
        });
        joystickArea.addEventListener('touchstart', (e) => {
            isDragging = true;
            joystickKnob.classList.add('dragging');
            joystickKnob.style.transform = 'none';
            handleJoystick(e);
        }, { passive: false });
        
        document.addEventListener('mousemove', handleJoystick);
        document.addEventListener('touchmove', handleJoystick, { passive: false });
        
        document.addEventListener('mouseup', resetJoystick);
        document.addEventListener('touchend', resetJoystick);
        
        // Slider Event Listener
        ['roll', 'pitch', 'yaw', 'height', 'legextend'].forEach(id => {
            const slider = document.getElementById(id);
            const valueEl = document.getElementById(id + 'Value');
            
            slider.addEventListener('input', () => {
                valueEl.textContent = slider.value;
                sendData();
            });
        });
        
        // Start
        connect();
    </script>
</body>
</html>
)rawliteral";

// WebSocket Event Handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", num);
        break;

    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
    }
    break;

    case WStype_TEXT:
    {
        // JSON parsen
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (!error)
        {
            // Werte in Struktur übernehmen
            robotControl.joystickX = doc["jx"] | 0;
            robotControl.joystickY = doc["jy"] | 0;
            robotControl.roll = doc["roll"] | 0;
            robotControl.pitch = doc["pitch"] | 0;
            robotControl.yaw = doc["yaw"] | 0;
            robotControl.height = doc["height"] | 175;
            robotControl.legextend = doc["legextend"] | 1200;

            // Debug Ausgabe
            Serial.printf("JoyX:%4d JoyY:%4d Roll:%3d Pitch:%3d Yaw:%3d Height:%3d Extend:%3d\n",
                          robotControl.joystickX,
                          robotControl.joystickY,
                          robotControl.roll,
                          robotControl.pitch,
                          robotControl.yaw,
                          robotControl.height,
                          robotControl.legextend);
        }
    }
    break;
    }
}

// Webseite ausliefern
void handleRoot()
{
    server.send(200, "text/html", index_html);
}

void setupWebServer()
{
    Serial.begin(115200);
    Serial.println("\n\nESP32 Robot Controller");
    Serial.println("======================");

    // Access Point starten (SSID sichtbar, max 4 Verbindungen)
    WiFi.softAP(ssid, password, 1, false, 4);
    IPAddress IP = WiFi.softAPIP();

    Serial.println("\nWiFi Access Point gestartet");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password: ");
    Serial.println(password);
    Serial.print("IP Adresse: ");
    Serial.println(IP);

    // WebServer Routes
    server.on("/", handleRoot);
    server.begin();
    Serial.println("WebServer gestartet auf Port 80");

    // WebSocket starten
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket Server gestartet auf Port 81");

    Serial.println("\n>> Verbinde dich mit dem WLAN und öffne http://192.168.4.1");
}