\page distributedsimulation Distributed Simulation

## Goals

* Simulation can be distributed among 1 or more processes
* These processes can be running on the same machine or different ones
* These processes must be kept in sync
* Results can't be interpolated or missed
* We should reduce the amount of duplicate effort across processes

## High-Level design

Each `ign gazebo` instance has the ability to run with the `--network-role` flag.
When the flag is present, the instance attempts to join a distributed simulation
environment by utilizing `ign-transport`. Ign-transport is used to register and
track available peers, as well as synchronize clock and state among multiple
distributed environment participants.

Distributed environment participants can take one of the following roles.

* Primary - responsible for distributing work and synchronizing clocks among the
            other participants
* Secondary - responsible for receiving work from the primary instance and
              executing physics and sensor simulation.  Results are reported
              back to the Primary.

The distribution of simulation work utilizes the concept of performers in
order to set where physics simulation will occur. A performer is an additional
annotation in an SDF file which marks each model that will be a performer.
There can be 0 to N performers being simulated at an instance at a time.
Each level can only be simulated by one secondary at a time, so performers
in the same level are always in the same instance. If there are more levels
active than instances, multiple levels will be allocated to each secondary.

## Assumptions

* When executing in a distributed environment, each `ign gazebo` instance only
  has one `SimulationRunner` instance, which means that instance is incapable
  of simulating multiple worlds.

* Distributed lockstep - all simulation runners step at the same time. If a
  particular instance is running slower than the rest, it will have an
  impact on the total simulation throughput.

* Fixed secondaries - all simulation runners have to be defined ahead of time.
  If a secondary joins or leaves the graph after simulation has started, simulation
  will terminate.

## Execution flow

### Configuration and launch

Multiple `ign gazebo` executables are started on the same local area network,
each with the `--network-role` flag set.

#### Command line options

The primary instance will read several command line options to dictate its behavior.

* **--network-role=primary** - Dictates that the role of this
    participant is a Primary. Capitalization of "primary" is not important.
* **--network-secondaries=<N>** - The number of secondaries expected
    to join. Simulation will not begin until **N** secondaries have been
    discovered.

The secondary instances will only read the role command line option

* **--network-role=secondary** - Dictates that the role of this
    participant is a Secondary. Capitalization of "secondary" is not important.

#### Environment variables

**WARNING:** Environment variables for distributed simulation configuration
are deprecated in version 2.x.x of Ignition Gazebo. Please use the
command-line options instead.

The primary instance will read several environment variables to dictate its behavior.

* **IGN_GAZEBO_NETWORK_ROLE=PRIMARY** - Dictates that the role of this
    participant is a Primary. Capitalization of "primary" is not important.
* **IGN_GAZEBO_NETWORK_SECONDARIES=<N>** - The number of secondaries expected
    to join. Simulation will not begin until **N** secondaries have been
    discovered.

The secondary instances will only read the role environment variable

* **IGN_GAZEBO_NETWORK_ROLE=SECONDARY** - Dictates that the role of this
    participant is a Secondary. Capitalization of "secondary" is not important.

### Discovery

Once the `ign gazebo` instance is started, it will begin a process of
discovering peers in the network. Each peer will send an announcement in
the `/announce` topic when it joins or leaves the network, and also
periodically sends a heartbeat on `/heartbeat`.

Simulation is allowed to begin once each secondary has discovered the
primary and the primary has discovered the correct number of secondaries.

If at any time the primary or any secondaries leave the network, either
intentionally (through shutdown) or unintentionally (segfault or network
issues), then the rest of the simulation graph will get a signal and shut down
safely.

There are two possible signals that can be received. The first is an intentional
announcement from a network peer that it is shutting down. The second is when
a peer fails to receive a heartbeat from another peer after a specified
duration. Both of these signals will cause the termination of the simulation.

### Distribution

Performers are distributed across secondaries in a way that:

* Each level can only be simulated by one secondary at a time.
* The number of idle secondaries is minimized.

The primary keeps all performers loaded, but performs no physics simulation.

#### Initial

After discovery, the `NetworkManager` works on the initial distribution of
performers across the network graph according to the levels they're in.

1. Group performers according to the levels they're in.
1. Distribute levels across secondaries in a round-robin fashion.
1. Distribute remaining performers, which aren't in any levels, in a
round-robin fashion.

If there are more levels with performers than secondaries, then some secondaries
will receive multiple levels and performers. On the other hand, if performers
are located in less levels than secondaries, some secondaries will be left idle.

#### Redistribution

As simulation proceeds and performers move across levels, their affinities will
be updated as part of the message sent on the `/step` topic.

* If a performer enters a level which is being simulated by another secondary,
either the new or the old performer must be moved. The choice of who to move
is made in a way that the least performers need to be moved.

* When a performer enters a level that was empty, the performer is moved to an
idle secondary, if available.

As an example, consider a situation where there are:

* 3 secondaries: `S1`, `S2` and `S3`
* 3 levels: `L1`, `L2` and `L3`
* 3 performers: `P1`, `P2` and `P3`

Initially:

* Performer `P1` is in level `L1`, simulated by secondary `S1`.
* Performers `P2` and `P3` are in level `L2`, simulated by secondary `S2`.
* `L3` is empty.
* `S3` is idle.

Some possible moves, all starting from the initial situation:

* `P1` enters `L3`: no reassignments, `S1` is simulating `L3` and `S2` is
simulating `L2`.
* `P1` enters `L2`: `P1` is moved to `S2` and `S1` is idle.
* `P2` and `P3` enter `L1` at the exact same iteration: `P1` is moved
to `S2` and `S1` is idle.
* `P3` enters `L3`: `P3` is moved to `S3`.
* `P2` enters `L1`, `P2` is moved to `S1`.

### Stepping

Stepping happens in 2 stages: the primary update and the secondaries update,
according to the diagram below:

<img src="https://bytebucket.org/ignitionrobotics/ign-gazebo/raw/default/tutorials/files/distributed_step.png"/>

1. The primary publishes a `SimulationStep` message on the `/step` topic,
containing:

    * The current sim time, iteration, step size and paused state.
    * The latest secondary-to-performer affinity changes.
    * The updated state of all performers which are changing secondaries.

1. Each secondary receives the step message, and:

    * Loads / unloads performers according to the received affinities
    * Runs one simulation update iteration
    * Then publishes its updated  performer states on the `/step_ack` topic.

1. The primary waits until it gets step acks from all secondaries.

1. The primary runs a step update:

    * Update its state with the states received from secondaries.
    * The `LevelManager` checks for level changes according to these new states
    * The `SceneBroadcaster` plugin publishes an updated scene to the GUI
      for any level changes.

1. The primary initiates a new iteration.

### Interaction

All interaction with the simulation environment should happen via the same
topics that are used for non-distributed simulation, which should all be
provided by the primary. Therefore, play/pause and GUI functionality all
interact with the simulation primary instance, which in turn propagates the
commands to the secondaries.

