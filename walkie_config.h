#pragma once

// Flash this sketch twice: leave true for the access-point CoreS3,
// then set false for the CoreS3 that joins that access point.
static constexpr bool DEVICE_IS_AP = true;

// Both CoreS3 units must use the same SSID and password.
// Replace this placeholder with your own 8+ character password before flashing.
static constexpr char WIFI_SSID[] = "CoreS3Walkie";
static constexpr char WIFI_PASSWORD[] = "REPLACE_WITH_YOUR_8PLUS_CHAR_PASSWORD";
static constexpr int WIFI_CHANNEL = 6;
