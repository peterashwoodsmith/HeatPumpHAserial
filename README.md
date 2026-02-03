This is a Zigbee end point controller for a Mitsubishi Heat Pump via its CT105 serial port.
For full details on code/hardware see the HeatPumpHAserial.ino code. A simple wiring diagram 
is provided in Wiring.pdf however this is based on my heat pumps and I make no guarantee your
wiring will be the same so you must measure carefully to determine if this is appropriate for
your devices.

Also included are a trivial little box and lid that conveniently fit the hardware on the side
of the heat pump that you can 3d print or of course any plastic box of suitable dimensions would work.

Ive also added the YAML code for the dashboard that I used for a single device and its clusters.
Since the zigbee chosen icons etc. are not good this overwrites those defaults and creates an 'ok'
interface however much more could be done with a markup or something more flexible but there are 
only so many hours in a day.

See the comments in the HeatPumpHAserial.ino for full details
