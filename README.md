# Antrum MK 1

## Inspiration

Antrum was inspired by a simple goal: helping protect people in high-risk underground environments before a rescue ever becomes necessary. We wanted to support cave explorers and miners who can become disoriented, separated, or trapped in places where visibility is low, communication is limited, and every wrong turn increases danger. The story of the Thai youth soccer team trapped in a cave for an extended period was a major point of inspiration for us. Their rescue was extraordinary, but it also showed how devastating it can be when people lose their bearings in an environment that is difficult to navigate and even harder to search. With Antrum, we wanted to build something preventative: a device that helps users keep track of their partner’s location in the cave and switch to a mode that guides them back toward the cave entrance from the point where recording began.

## What it does

1. Pair both devices before going into cave.
2. Strap it on to your leg, then go caving.
3. If you get lost, use the LED indicators to find where your caving partner is.
4. If you want to know the way back to the start, switch modes.
5. Once you surface; exploration data is sent to the cloud, then you can visualise both your paths on the webapp (Altrum DASH).

## How we built it

Our team was able to move quickly because we had a strong mix of complementary skills. Chris led the hardware coding and embedded implementation in C, Jenul focused on full-stack development and maintainable software architecture, Maria brought expertise in electrical engineering, PCB design, and 3D printing, and Manan contributed a strong mathematical foundation that helped shape the positioning logic behind the device. That combination allowed us to work in parallel, make decisions quickly, and connect hardware, software, and modelling into one system.

On the hardware side, we rapidly researched the components we needed and designed the device around an ESP32 as the embedded master controller. We used the XC4508 for 2.4 GHz radio communication between paired devices, and the ICM20948 as the IMU to capture movement data for cavers and miners. To estimate position, we built our tracking approach around dead reckoning and ZUPT (Davis, 1607/1880; Foxlin, 2005), but the maths went further than simple integration. We used zero-velocity updates (ZUPT; Foxlin, 2005) to detect when a device was effectively stationary by checking whether the gyroscope magnitude satisfied $\|\omega\| < \omega_{\text{zupt}}$ and whether acceleration stayed close to gravity using $\left|\|a\| - g\right| < \epsilon_a$. When those conditions held, we set velocity to zero, held position at the last trusted point, and prevented stationary drift from accumulating.

$$
\|\omega\| < \omega_{\text{zupt}}, \qquad \left|\|a\| - g\right| < \epsilon_a \;\Longrightarrow\; v = 0
$$

We also added big-movement gating and small-movement rejection so that only believable motion was integrated, while small jitter and noise were damped out before they could corrupt the trajectory. To make partner tracking meaningful, each device shared its position, yaw, initial yaw, and timestamp over the RF24 link, allowing us to compute a shared yaw offset $\Delta \psi = \psi_{\text{peer},0} - \psi_{\text{local},0}$, rotate coordinates into a common frame, and then evaluate relative position as $p_{\text{rel}} = p_{\text{peer}} - p_{\text{local}}$.

$$
\Delta \psi = \psi_{\text{peer},0} - \psi_{\text{local},0}, \qquad p_{\text{rel}} = p_{\text{peer}} - p_{\text{local}}
$$

On the software side, we built the web application early using mocked data so frontend progress would not be blocked by hardware bring-up. We kept everything in a monorepo so the whole team could collaborate efficiently, used TypeScript and Hono for a fast backend, and chose Angular for a clean and maintainable frontend architecture. To reduce deployment friction during the hackathon, we set up CI/CD early, deployed the backend to a Google Cloud virtual machine, and hosted the frontend on Vercel.

## Challenges we ran into

## Accomplishments that we're proud of

## What we learned

One of our biggest technical lessons was how much real-world constraints change your ideal design. Chris learned that the RF24 module behaved very differently from other radios he had used before, especially because it could not transmit and receive simultaneously. That limitation forced us to redesign communication around a master-slave packet exchange instead of a simpler synchronization model. Maria learned not only the value of structuring CAD work with separate part files in SolidWorks so enclosure designs stay editable, but also how much product design is a balance between the ideal form, the physical limitations of PLA printing, and the time constraints of the project itself. Jenul learned a great deal from the hardware-software integration process, especially around cleaning, compressing, and transforming noisy IMU output into data that could be rendered meaningfully in the frontend, drawing on practical mathematical techniques for filtering, approximation, and more efficient representation. Manon learned how quickly neat theoretical models break down in physical environments, and how important it is to adapt mathematical methods to interference, sensor drift, and imperfect real-world conditions.

As a team, we learned how powerful strong cross-functional collaboration can be under tight time pressure. This project only worked because hardware, firmware, mathematics, CAD, and web development were constantly informing each other. Chris and Jenul aligned embedded output with the web platform, Maria coordinated hardware and enclosure design, and Manon helped translate complex mathematical ideas into something the software could actually use. None of us started with deep knowledge of every part of the system, but by working closely and teaching each other as we went, we were able to build a much more complete and functional product than any one discipline could have produced alone.


## What's next for Antrum MK 1

For a Mark 2 version of Antrum, we want to develop a fully waterproof casing so the device can reliably support underwater cave divers as well. That next iteration would also replace our current 2.4 GHz RF approach with UWB, since standard RF communication at that frequency is not suitable underwater. We are confident the core idea can extend to that environment, but the hackathon timeframe was too short for us to redesign the communications layer and properly engineer, test, and validate an underwater-ready version in this iteration.

## References

### Knowledgebase

#### Dead reckoning method
- Davis, J. (1607/1880). The seaman’s secrets. In A. H. Markham (Ed.), *The voyages and works of John Davis, the navigator* (pp. 230–337). Hakluyt Society. https://www.spirasolaris.ca/sbb9d1.pdf

#### ZUPT (Zero-Velocity Update method for IMUs)
- Foxlin, E. (2005). Pedestrian tracking with shoe-mounted inertial sensors. *IEEE Computer Graphics and Applications, 25*(6), 38–46.

### Video references

- Good Morning America. (2023, September 14). Mark Dickey speaks out about rescue from Turkish cave l GMA [Video]. YouTube. https://www.youtube.com/watch?v=CsPicsmCib0

- 7NEWS. (2025, December 1). Cave rescue expert called for diver recovery [Video]. YouTube. https://www.youtube.com/watch?v=ekqP49hA5rI

- TODAY. (2023, November 29). Rescued Thai cave survivor shares update 5 years later [Video]. YouTube. https://www.youtube.com/watch?v=HQukKjfrN18

- NBC News. (2018, December 7). Flashback: How the Chilean miners rescue happened | NBC Nightly News [Video]. YouTube. https://www.youtube.com/watch?v=jDxf6MT-LaM

### Documentation

- Angular. (n.d.). Angular Documentation. Retrieved March 14, 2026, from https://angular.dev/overview

- Microsoft. (n.d.). The TypeScript handbook. Retrieved March 14, 2026, from https://www.typescriptlang.org/docs/handbook/intro.html

- Hono. (n.d.). Hono documentation. Retrieved March 14, 2026, from https://hono.dev/docs/

- Anthropic. (n.d.). Common workflows. In Claude Code Docs. Retrieved March 14, 2026, from https://docs.anthropic.com/en/docs/claude-code/common-workflows

- PlatformIO. (n.d.). PlatformIO documentation. Retrieved March 14, 2026, from https://docs.platformio.org/en/latest/

- Dassault Systèmes. (n.d.). SOLIDWORKS Web Help. Retrieved March 14, 2026, from https://help.solidworks.com/
