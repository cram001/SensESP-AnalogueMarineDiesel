#pragma once
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

/**
 * Ensure /config/metadata.json exists and contains the required
 * Signal K metadata used by the diesel engine monitor.
 * 
 * Called BEFORE SensESPAppBuilder builds the app.
 */
inline void ensure_metadata_file() {
  const char* path = "/config/metadata.json";

  // Mount LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount failed");
    return;
  }

  // If file already exists, do nothing
  if (LittleFS.exists(path)) {
    Serial.println("metadata.json exists — OK");
    return;
  }

  Serial.println("metadata.json missing — creating default metadata…");

  // Create /config directory if missing
  if (!LittleFS.exists("/config")) {
    LittleFS.mkdir("/config");
  }

  // Create file
  File f = LittleFS.open(path, "w");
  if (!f) {
    Serial.println("ERROR: Could not create metadata.json");
    return;
  }

  // The required metadata for your project
  const char* json =
      "{\n"
      "  \"propulsion.engine.coolantTemperature\": {\n"
      "    \"units\": \"K\",\n"
      "    \"displayName\": \"Coolant Temp\"\n"
      "  },\n"
      "  \"propulsion.engine.revolutions\": {\n"
      "    \"units\": \"Hz\",\n"
      "    \"displayName\": \"Engine RPM\"\n"
      "  },\n"
      "  \"propulsion.engine.fuel.rate\": {\n"
      "    \"units\": \"m3/h\",\n"
      "    \"displayName\": \"Fuel Flow\"\n"
      "  },\n"
      "  \"propulsion.engine.runTime\": {\n"
      "    \"units\": \"h\",\n"
      "    \"displayName\": \"Engine Hours\"\n"
      "  }\n"
      "}\n";

  f.print(json);
  f.close();

  Serial.println("metadata.json created successfully.");
}
