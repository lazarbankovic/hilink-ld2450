# hilink-ld2450
This is a serial driver for Hilink's ld2450 radar module. 

Usage is really simple:  

First of all create an instance and give it the Serial port which is connected to the ld2450 module.  
Then also pass the handler for a data frame which comes from the ld module.  
```cpp
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
```  
The other way to do it is just by passing the Serial instance and a simple handler function.  
```cpp
#include "Ld2450.h"

/* Create data handler function */
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

/* Pass the serial instance and the data handler function to ld2450 driver instance. */
hilink::Ld2450 ld2450(Serial2, callback);
```
In setup phase, we init serial for the PC communication. Then, we init the ld2450 driver (which will just init the Serial for the ld2450 communication).
Calling .startData() will start receiving the data from the module.
```cpp
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
```
In loop phase, we need to peridically call .tick() method.
```cpp
void loop()
{
    /* Tick method needs to be called periodically */
    ld2450.tick();
}
```
Examples on how to configure module and how to read configuration from the module can be found in the [examples](examples/) folder.  
Documentation regarding protocol can be found in the [doc](doc/) folder. It's especially useful if you want to know the format of the detection coordinate system.

