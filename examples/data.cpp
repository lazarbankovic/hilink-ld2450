/**
 * This example shows how to capture data from ld2450.
 *
 * Two serials are used:
 * Serial - communication with the host for printing data
 * Serial2 - communication with the ld2450 radar module
 *
 */

#include "Ld2450.h"

hilink::Ld2450 ld2450(Serial2,
                      [](hilink::Ld2450::SensorTarget target,
                         const hilink::Ld2450::SensorData &data)
                      {
                          Serial.printf("[target %d] x: %d, y: %d, speed: %d, resolution: %d\n",
                                        target,
                                        data.x,
                                        data.y,
                                        data.speed,
                                        data.resolution);
                      });

/* Simple callback can be used as well:

    void callback(hilink::Ld2450::SensorTarget target,
                  const hilink::Ld2450::SensorData &data)
    {
        Serial.printf("[target %d] x: %d, y: %d, speed: %d, resolution: %d\n",
                      target,
                      data.x,
                      data.y,
                      data.speed,
                      data.resolution);
    }
    hilink::Ld2450 ld2450(Serial2, callback);
*/

void setup()
{
    /* Init serial for the host communication */
    Serial.begin(115200);

    /* Init the ld2450 driver */
    ld2450.init();

    /* Start parsing data from ld serial */
    ld2450.startData();

    /* After this call the callback registered in the constructor starts executing */
}

void loop()
{
    /* Tick method needs to be called periodically */
    ld2450.tick();
}
