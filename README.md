# btsdk-drivers

### Overview

This repo contains driver libraries used in BTSDK 2.0 and higher. The libraries included in this repo are:

* hsl\_rgb\_lib<br/>
    * Library to convert HSL Color to RGB Color values<br/>


* thermistor\_ncp15xv103\_lib<br/>
    * Library providing API implementation for NCP15XV103E03RC thermistor<br/>


* thermistor\_ncu15wf104\_lib<br/>
    * Library providing API implementation for NCU15WF104F60RC thermistor<br/>


* ambient\_light\_sensor\_lib<br/>
    * Ambient Light Sensor library for Mesh kit (CYBT-213043-MESH)<br/>


* pir\_motion\_sensor\_lib<br/>
    * PIR (Passive Infrared Motion Sensor) library for Mesh kit (CYBT-213043-MESH)<br/>


* button\_manager<br/>
    * Library providing Button Management<br/>


* led\_manager<br/>
    * LED Management Support<br/>


* nvram\_emulation<br/>
    * NVRAM emulation in RAM for kits like CYW955513EVK-01 without flash access<br/>
    This library provides APIs to allow platforms without flash to emulate NVRAM in RAM,
    which allows sample applications to demonstrate functionality normally requiring NVRAM access,
    for example link key storage and retrieval.
    This can also be used as a starting point for solutions that will use an external host to store
    and retrieve data from host storage.
    To include NVRAM emulation support add COMPONENTS+=nvram_emulation to the makefile.
    In C code where the wiced_hal_\*_nvram API will be used, make changes as follows:
```js
    /* add header */
    #ifdef COMPONENT_nvram_emulation
    #include "nvram_emulation_mem.h"
    #endif
    /* initialize the library before using */
    #ifdef COMPONENT_nvram_emulation
        nvram_emulation_mem_init();
    #endif
```
