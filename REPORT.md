# Traffic Light Controller with Pedestrian Crossing

This project is a working prototype for a four-way intersection controller using an Arduino Uno in Tinkercad Circuits. It models a realistic traffic sequence with safe pedestrian crossing behaviour.

## Design summary
- Two traffic streams are controlled:
  - North-South (NS)
  - East-West (EW)
- Only one stream is green or yellow at any time.
- There is a brief all-red clearance period between streams.
- Pedestrian requests are stored in software using latching flags so they are not lost while traffic is still moving.
- Pedestrian crossing is served during the next safe all-red interval, before the next traffic stream starts moving.

## Timing choices
The controller uses the following timings:
- Green: 6000 ms (6s)
- Yellow: 2000 ms (2s)
- All-red clearance: 1500 ms (1.5s)
- Pedestrian WALK: 5000 ms (5s)

These values are realistic for a small urban intersection simulation and satisfy the assignment requirement of a short but clear clearance phase.

## Pedestrian indication choice
This design uses dedicated pedestrian LEDs on each side of the crossing:
- NS WALK = green LED
- NS DON'T WALK = red LED
- EW WALK = green LED
- EW DON'T WALK = red LED

This was chosen because it makes the crossing state visually unambiguous and avoids confusing the pedestrian signal with the normal traffic-light LEDs. It also allows the traffic lights to remain logically separated from the walk signal, which is clearer in simulation and in a real system.

## Safety logic
The system follows a safe traffic-light sequence:
1. Current stream shows green.
2. Current stream shows yellow.
3. All traffic lights show red for a short clearance period.
4. If a pedestrian request exists for that stream or the next stream, a WALK indication is shown while all vehicle lights remain red.
5. The opposite traffic stream then receives green.

This prevents a pedestrian crossing from being allowed while any conflicting vehicle phase is green.

## Files
- `traffic_light_controller.ino` — implementation of the controller state machine
- `README.md` — quick overview
