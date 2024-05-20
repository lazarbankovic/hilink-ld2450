
/**
 * This example shows how to configure and get configuration from the ld2450
 * module.
 *
 * Two serials are used:
 * Serial - communication with the host for printing data
 * Serial2 - communication with the ld2450 radar module
 *
 * To configure the module, always follow these steps:
 *
 * 1. Enter the config mode.
 * 2. Do the configuration. There is a short list of config methods which can be
 *    used. Some are presented here, and some are not.
 * 3. Exit the config mode.
 */

#include "Ld2450.h"

hilink::Ld2450 ld2450(Serial2,
                      [](hilink::Ld2450::SensorTarget target,
                         const hilink::Ld2450::SensorData& data) {});

void printData(hilink::Ld2450::Zone& zone) {
  /* Printing in the format which was used for the (mobile) app provided by
   * the hilink */
  Serial.printf("x1 %d y1 %d w %d h %d \n",
                5500 + zone.x1,
                zone.y1,
                abs(zone.x1 - zone.x2),
                abs(zone.y1 - zone.y2));
}

void setup() {
  /* Init serial for the host communication */
  Serial.begin(115200);

  /* Init the ld2450 driver */
  ld2450.init();

  /* Prior to calling any of the configuration methods, set ld2450 into the
  config mode, otherwise the config command will fail. After config mode is set,
  data is turned off. */
  if (!ld2450.setConfigMode(true)) {
    Serial.println("Error setting config mode!");
  }

  /* Turn bluetooth on.
  The bt needs to be turned on, for some reason config commands fail if it's
  off. */
  if (!ld2450.setBluetooth(true)) {
    Serial.println("Error turning bt on");
    return;
  }

  /* Reset the module */
  if (!ld2450.reset()) {
    Serial.println("Error resetting module");
  }

  /* Get fw version string */
  String version;
  if (!ld2450.getFwVersion(version)) {
    Serial.println("getting version failed");
  } else {
    Serial.println("Fw Version: " + version);
  }

  /* Get bt mac address */
  uint8_t mac[6];
  if (!ld2450.getMac(mac)) {
    Serial.println("Error getting mac");
  } else {
    printf("%0.2X:%0.2x:%0.2x:%0.2x:%0.2x:%0.2x",
           mac[0],
           mac[1],
           mac[2],
           mac[3],
           mac[4],
           mac[5]);
  }

  /* Read the capture zones.
  Check the ld2450 documentation distributed with this driver for more
  information about what do the numbers mean*/
  hilink::Ld2450::Zones zones;
  if (!ld2450.getZones(zones)) {
    Serial.println("Error getting zones");
  } else {
    /* So basically a zone is defined with two cordinates (x1,y1) and (x2,y2).
     * Those are diagonal coordinates of a rectangular area, so 'top left' and
     * 'bottom right'.
     * The sensor sits in the middle, so (0,0) and the coordinates are relative
     * to it */
    printData(zones.one);
    printData(zones.two);
    printData(zones.three);
  }

  /* Exit the config mode.
  After exiting the config mode, data can be turned on */
  if (!ld2450.setConfigMode(false)) {
    Serial.println("Error exiting config mode");
  }
}

void loop() {
  /* Nothing to call here, we're just configuring the module in this example */
}
