/*
 * DroneController - Arduino UNO R4 WiFi
 * Step 4: Parse and validate incoming control packets
 *
 * Wire format:
 *   CTRL,SEQ=<n>,T=<0-1000>,Y=<0-1000>,P=<0-1000>,R=<0-1000>,A=<0|1>,ES=<0|1>*<checksum>
 *
 * Validation performed before ANY value is accepted:
 *   - Must start with "CTRL,"
 *   - Must contain a '*' separating payload from checksum
 *   - Checksum must match (8-bit XOR of payload chars, 2 hex digits)
 *   - All 6 fields (SEQ,T,Y,P,R,A,ES... actually 7 incl SEQ) must be
 *     present and parseable as integers
 *   - T,Y,P,R must be in 0-1000; A,ES must be 0 or 1
 * A packet failing ANY check is dropped entirely - no partial apply.
 */

#include <WiFiS3.h>

const char* AP_SSID = "DroneController";
const char* AP_PASSWORD = "drone1234";
const int AP_CHANNEL = 6;
const int TCP_PORT = 5555;

WiFiServer server(TCP_PORT);
WiFiClient client;

// Telemetry timing - sent independently of control packet reception,
// at a fixed rate, whenever a client is connected.
unsigned long lastTelemetryMillis = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 100; // 10 Hz
unsigned long telemetrySeq = 0;

// Latest validated control state. Step 7 adds the fail-safe watchdog
// that forces this back to safe values on timeout.
struct ControlState {
  unsigned long seq = 0;
  int throttle = 0;    // 0-1000
  int yaw = 500;        // 0-1000, 500 = center
  int pitch = 500;
  int roll = 500;
  bool armed = false;
  bool emergencyStop = false;
  unsigned long lastValidPacketMillis = 0;
};

ControlState controlState;

// Stats for debug visibility.
unsigned long packetsReceived = 0;
unsigned long packetsRejected = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=== DroneController - Arduino UNO R4 WiFi ===");
  Serial.println("Starting WiFi Access Point...");

  int status = WiFi.beginAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  if (status != WL_AP_LISTENING) {
    Serial.println("FATAL: Failed to start Access Point. Halting.");
    while (true) {
      delay(1000);
    }
  }

  delay(2000); // let the AP fully settle before starting the TCP server

  IPAddress apIP = WiFi.localIP();
  Serial.print("Access Point started. SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Arduino IP address: ");
  Serial.println(apIP);
  Serial.print("TCP port: ");
  Serial.println(TCP_PORT);
  Serial.println();

  server.begin();
  Serial.println("TCP server listening for a connection...");
}

/**
 * Computes the 8-bit XOR checksum of a payload string, returned as
 * 2 uppercase hex digits, matching PacketBuilder.kt on the Android side.
 */
String computeChecksum(const String& payload) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < payload.length(); i++) {
    checksum ^= (uint8_t)payload[i];
  }
  char buf[3];
  sprintf(buf, "%02X", checksum);
  return String(buf);
}

/**
 * Builds and sends one telemetry packet over the given client.
 *
 * NOTE: battery voltage (V) and percentage (B) are PLACEHOLDER values
 * in this step - real battery sensing via voltage divider is Step 6.
 * Using a fixed plausible value now lets us verify the telemetry
 * pipeline end-to-end before wiring in real hardware.
 */
void sendTelemetry(WiFiClient& c, const ControlState& state) {
  float placeholderVoltage = 11.42; // stub - Step 6 replaces this
  int placeholderPercent = 78;       // stub - Step 6 replaces this

  unsigned long uptime = millis();

  String payload = "SEQ=" + String(telemetrySeq++) +
                    ",V=" + String(placeholderVoltage, 2) +
                    ",B=" + String(placeholderPercent) +
                    ",ARM=" + String(state.armed ? 1 : 0) +
                    ",FS=" + String(state.emergencyStop ? 1 : 0) +
                    ",UP=" + String(uptime);
                    
  String checksum = computeChecksum(payload);
  String packet = "TEL," + payload + "*" + checksum;

  c.println(packet);
}

/**
 * Extracts the integer value following "KEY=" up to the next ',' or
 * end of string. Returns false if the key isn't found or the value
 * isn't a valid integer.
 */
bool extractIntField(const String& payload, const char* key, long& outValue) {
  String searchKey = String(key) + "=";
  int keyIndex = payload.indexOf(searchKey);
  if (keyIndex == -1) return false;

  int valueStart = keyIndex + searchKey.length();
  int valueEnd = payload.indexOf(',', valueStart);
  if (valueEnd == -1) valueEnd = payload.length();

  String valueStr = payload.substring(valueStart, valueEnd);
  if (valueStr.length() == 0) return false;

  // Validate every character is a digit (or leading '-' for SEQ, though
  // SEQ should never be negative in practice - reject if it is).
  for (size_t i = 0; i < valueStr.length(); i++) {
    char c = valueStr[i];
    if (!isDigit(c) && !(i == 0 && c == '-')) return false;
  }

  outValue = valueStr.toInt();
  return true;
}

/**
 * Parses and validates one full line as a CTRL packet. Returns true
 * and fills outState if valid; returns false and leaves outState
 * untouched if the packet fails any check.
 */
bool parseControlPacket(const String& line, ControlState& outState) {
  if (!line.startsWith("CTRL,")) return false;

  int starIndex = line.lastIndexOf('*');
  if (starIndex == -1) return false;

  String payload = line.substring(5, starIndex); // strip "CTRL," prefix
  String receivedChecksum = line.substring(starIndex + 1);
  receivedChecksum.trim();

  String expectedChecksum = computeChecksum(payload);
  if (receivedChecksum != expectedChecksum) {
    return false;
  }

  long seq, t, y, p, r, a, es;
  if (!extractIntField(payload, "SEQ", seq)) return false;
  if (!extractIntField(payload, "T", t)) return false;
  if (!extractIntField(payload, "Y", y)) return false;
  if (!extractIntField(payload, "P", p)) return false;
  if (!extractIntField(payload, "R", r)) return false;
  if (!extractIntField(payload, "A", a)) return false;
  if (!extractIntField(payload, "ES", es)) return false;

  // Range validation - reject rather than clamp, per the spec: an
  // out-of-range value indicates a corrupt or malicious packet, not
  // something to silently "fix."
  if (t < 0 || t > 1000) return false;
  if (y < 0 || y > 1000) return false;
  if (p < 0 || p > 1000) return false;
  if (r < 0 || r > 1000) return false;
  if (a != 0 && a != 1) return false;
  if (es != 0 && es != 1) return false;

  outState.seq = (unsigned long)seq;
  outState.throttle = (int)t;
  outState.yaw = (int)y;
  outState.pitch = (int)p;
  outState.roll = (int)r;
  outState.armed = (a == 1);
  outState.emergencyStop = (es == 1);
  outState.lastValidPacketMillis = millis();

  return true;
}

void loop() {
  if (!client || !client.connected()) {
    WiFiClient newClient = server.accept();
    if (newClient) {
      client = newClient;
      Serial.println(">>> Phone connected!");
    }
  }

  if (client && client.connected()) {
    while (client.available() > 0) {
      String line = client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;

      ControlState parsed;
      if (parseControlPacket(line, parsed)) {
        controlState = parsed;
        packetsReceived++;

        Serial.print("OK  SEQ=");
        Serial.print(controlState.seq);
        Serial.print(" T=");
        Serial.print(controlState.throttle);
        Serial.print(" Y=");
        Serial.print(controlState.yaw);
        Serial.print(" P=");
        Serial.print(controlState.pitch);
        Serial.print(" R=");
        Serial.print(controlState.roll);
        Serial.print(" A=");
        Serial.print(controlState.armed);
        Serial.print(" ES=");
        Serial.println(controlState.emergencyStop);
      } else {
        packetsRejected++;
        Serial.print("REJECTED: ");
        Serial.println(line);
      }
    }
  }

  // Send telemetry at a fixed rate, independent of control packet
  // reception - this is effectively our heartbeat back to the phone.
  if (client && client.connected()) {
    unsigned long now = millis();
    if (now - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
      lastTelemetryMillis = now;
      sendTelemetry(client, controlState);
    }
  }

  static bool wasConnected = false;
  bool isConnected = client && client.connected();
  if (wasConnected && !isConnected) {
    Serial.println("<<< Phone disconnected.");
  }
  wasConnected = isConnected;
}