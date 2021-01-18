
In order to correctly run the firmware on Renode platform some important steps must be done.

1. In "stm32f4discovery_modified.resc" file subtitute in line 23 the absolute path of the .axf file located in MDK-ARM/TEST_RENODE folder and save.
2. In Renode directory (C:/programs/Renode) add the "stm32f4discovery_modified.resc" file.
3. Go to Renode/boards directory and substitute the file "stm32f4_discovery-additional_gpios.repl" with the one in this folder.
4. In Renode/boards open the file "stm32f4_discovery.repl", substitute the first line with the following one:
using "platforms/cpus/stm32f429.repl"
5. Launch Renode application and on the monito CLI type:
include @stm32f4_discovery_modified.resc
6. The machine is correctly configured.
7. Type "start".

For the terminal emulation run PuTTY (or anyother application supporting telnet) and configure a telnet communication with host ```name:localhost``` and ```port:3456``` 

If when performing step 5 Renode returns some errors concerning the sensors, probably the recompiled version (supporting BMP180) must be installed.
In this case install the recompiled version using ```renode_1.11.0.msi``` in the current folder and re-run all the steps 1-7.
