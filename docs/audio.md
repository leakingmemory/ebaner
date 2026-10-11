# Sound

All the sound is synthesised in real time from what the simulation is doing
(`src/Audio.cpp`). It needs PulseAudio or PortAudio at build time; without either the
simulator is silent.

## Brake air

A hiss whose loudness follows the airflow: a subdued charge on applying and a louder,
brighter vent on releasing, fading as the pressures equalise and with the camera's
distance from the bogies. A valve click marks each change of the handle or the safety
device.

## Engines

Each diesel has an idle voice built from that engine's own figures: the firing rate
follows from its cylinders and cycle, and its beat, loudness, low-end weight and bark
come with it.

| | Engine | Idle | Character |
|---|---|---|---|
| Class 93 | two Cummins, 6 cylinders, four-stroke | 700 rpm | a muffled drone, beating 35 times a second, heavily low-passed for the insulated body |
| Di 4 | EMD 645, V16 two-stroke | 315 rpm | beating 84 times a second, with strong energy at half the firing rate and below; about 2–2.8× as loud as the railcar |
| CD 312 | EMD 710, V16 two-stroke | 200 rpm | the Di 4's kind of voice, with less low-end weight, which would fall below 20 Hz |

The big diesels' voices also have:

- an **exhaust blast**, a pulse over about a third of each firing period;
- **firing-to-firing variation**, about a third of the amplitude;
- the **gear train and injectors**, a rattle at half the firing rate.

The voice fades in from cranking to the engine's own idle, so there is no combustion
sound while the starter turns it. The engines rev with the power, and a compressor
running adds a muffled pump hum and pulls its engine's idle down a little. Up to **six**
engine voices play at once, so a three-set Class 93 is heard as six engines.

## The Di 4's grid blower

The electric brake's grid fans are fed from the braking current, so they speed up the
harder the brake works. The roar builds over a couple of seconds, and coasts down when
the brake is notched off. It is a hum of four harmonics of the blade-passing tone, which
rises from about 214 Hz to 618 Hz across the notches, with the shaft's throb under it.
That the blower is current-fed is an assumption.

## Coupling and collision

One voice covers every contact between two trains. The faster the closing speed, the
lower the thud, the darker the noise and the longer the decay: a gentle coupling is a
bright clack of a fifth of a second, a collision over a second of low end. It is heard
from where the trains met.

## Rolling noise

The loudest thing a running train makes; above about 60 km/h it covers the engines.
It is synthesised from the wheel–rail contact.

- **Speed** sets the level at 30 · log₁₀(v), so four times the speed is eight times the
  amplitude, and raises the pitch from a low grumble at a crawl to a full rush at line
  speed. Below walking pace it is silent.
- **Axle load** makes it **darker** as well as louder: a heavier axle has a wider contact
  patch and lower-radiating wheels. From a bare wheelset (1.3 t an axle) to a Class 93
  (11.7 t) the spectral centroid falls from about 950 Hz to 690 Hz.
- **Friction.** Traction and running resistance roughen the roar. The friction brake
  adds a soft low rumble. The lateral load the cant doesn't take out drives a **curve
  squeal** around 2.1 kHz, only on curves tighter than about 300 m.
- **Switches.** The rail is continuously welded, so the knocks come at the switches: each
  wheel knocks once at each frog, spaced by the vehicle's axles. Impact noise grows more
  slowly with speed than rolling noise, so the knocks stand out at yard speed.

A derailed vehicle makes none of it.

## Checking it

Each part can be rendered to a WAV file, without a window:

| Variable | Renders |
|---|---|
| `EBANER_AUDIO_DUMP=<file>` | a brake sequence |
| `EBANER_AUDIO_DUMP_ENGINE=<file>` | an engine start, idle, compressor, full power and shutdown; `EBANER_AUDIO_ENGINE` picks the machine |
| `EBANER_AUDIO_DUMP_GRID=<file>` | the Di 4's electric brake applying |
| `EBANER_AUDIO_DUMP_IMPACT=<file>` | contacts across the range of closing speeds |
| `EBANER_AUDIO_DUMP_CROSSING=<file>` | a level crossing bell starting and stopping |
| `EBANER_AUDIO_DUMP_ROLLING=<file>` | rolling noise over speed, weight, curve and brake |

`ctest -R rolling-noise` tests the rolling noise.
