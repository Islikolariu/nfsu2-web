# Experimental FPV camera

This branch adds an optional Acro FPV camera to the local browser build of
NFSU2 Web. It uses the flight model from the Windows FPV prototype: body
rotation rates, motor response, thrust and drag. The camera is set to 46°
upward tilt and 105° FOV, with a 750 g drone.

## Try it

Build the game from your own compatible copy using the [upstream build
instructions](../../README.md#build). Start the server on port 8611:

```sh
web/start.sh --data "/path/to/Need for Speed Underground 2" --http-port 8611
```

Open `http://localhost:8611/?fpv=1`. Enter Free Roam, stop the car on a
level road and press **F8**. Keep the throttle at zero when you enter, then
raise it to take off. Press **F8** or **Esc** to return to the car.

| Input | Throttle | Yaw | Pitch | Roll |
| --- | --- | --- | --- | --- |
| Keyboard, with no controller connected | Hold **W** for power; release or press **S** to cut it | A / D | I / K | J / L |
| Standard gamepad | Left stick up, zero at the centre | Left stick left / right | Right stick up / down | Right stick left / right |
| DJI FPV RC 3 | Left stick up, zero at the bottom | Left stick left / right | Right stick up / down | Right stick left / right |

For a DJI radio with different axis order or direction, add
`&fpv_axes=1,2,4,-3` to the URL and adjust the four signed, one-based
indices for roll, pitch, yaw and throttle. If its browser name is generic,
`&fpv_pad=` accepts a substring of the reported name.

FPV is enabled only on localhost with the flag above. The camera stays
within 100 m of the car, where the game streams scenery. There is a floor
based on the car's height; buildings and terrain have no drone collision.
The drone has no visible model, motor audio or network presence in this
branch. Other players still see the car.

## Verify without game files

```sh
cc -std=c11 -O2 -Iruntime tests/fpv_acro_unit.c runtime/fpv_flight.c -lm -o /tmp/nfsu2-fpv-acro
/tmp/nfsu2-fpv-acro
cd web
npm ci
npm run check
node --test test/fpv.test.mjs
```
