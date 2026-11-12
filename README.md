# Adaptive Anti-Jam Controller for a Screw Press- Still in progress

An adaptive, load-based controller that detects and clears jams on a scaled
screw-press rig, and performs measurably better than an uncontrolled baseline.

![Simulation: no control vs with control] 

## Problem
During operation, screw presses often encounter jams. This is usually caused by accumulation of seed oil press cake at the discharge outlet during oil extraction. The operator would have to disassemble the machine to remove the accumulated cake, which is tedious and tiresome. Frequent jam occurrence reduces efficiency of the machine while also reducing the quality of oil extracted, as some oil cakebits escape as a result of the accumulation. 

## Approach
The controller monitors motor current and speed and runs this sequence:

RUN -> SLOW -> STOP -> REVERSE -> STOP -> RESUME (ramped) -> RUN

- Debounced overload detection
- A "jam gone?" check at reduced speed before committing to a reverse
- Ramped restart to limit the current spike
- A FAULT state after repeated failed attempts

## Results

### Simulation (illustrative model, assumed parameters)

![Simulation results]

A jam is introduced at 15 s and grows until 30 s. The uncontrolled machine
keeps drawing high current and ends up nearly stalled. The controller detects
the overload, slows down, stops, reverses, stops again and resumes with a
ramped restart. The jam returns once because material is still being fed, so
the sequence runs twice, and the machine then recovers fully.

| Metric (60 s run)                | No control | With control |
|----------------------------------|-----------:|-------------:|
| Peak current after startup       | 9.8 A      | 5.8 A        |
| Time above overload threshold    | 42.0 s     | 10.3 s       |
| Peak temperature                 | 39.8 °C    | 28.9 °C      |
| Mean forward speed after 15 s    | 37 %       | 67 %         |
| Final speed                      | 43 rad/s   | 193 rad/s    |

The ramped restart keeps the resume current spike under 6 A, compared with
about 14 A for an abrupt restart in an earlier version of the model.

**These numbers come from a model, not the machine.** The motor and jam
parameters are illustrative, and the model assumes that reversing clears the
jam. Testing that assumption is the purpose of the rig experiment.

### Rig results
*To be added as experiments are completed.*

## Author
Sogomo, [Kenyatta University], [2026]. 
