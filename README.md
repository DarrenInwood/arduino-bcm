# Arduino BCM System

This system is designed to provide 'home automation' specific to a campervan,
RV, or regular passenger car.

It was designed for older vehicles that don't have OBD-II or CAN-BUS.
Implementation should be a lot easier in vehicles implementing OBD-II as the
sensors etc are all available digitised.  However, many vehicle manufacturers
do not have details of their CAN protocols available, so deep integration with
these systems will probably involve reverse engineering the available CAN
signals.

CAN-BUS was chosen as it is ubiquitous in vehicles already, provides the sort of
broadcast messages needed for a distributed system, has a lot of complexity
abstracted out to the bus controller chips such as error detection and bus
collision detection, is cheap to obtain CAN controller modules, and there are
already a number of implementations of similar systems built using CAN that we
can leverage here.

## CAN-BUS systems, and OpenLCB

There are already a number of systems that allow Arduino development of home
automation systems.  I tried out Souliss (http://www.souliss.com) and VSCP
(http://vcsp.org) before settling on OpenLCB.

Souliss seemed ideal, but the codebase was written in a very confusing way in my
opinion.  There was no existing CAN driver for their vNet virtual network
implementation, and I found writing one was too difficult.

VSCP also seemed like it was great, but I just couldn't get the software you
need to run on any of my computers.  Also, it seemed like you would have to
purchase a commercial USB->CAN adapter and install drivers on your system to
use the software.

In the end I settled on OpenLCB as it followed the 'distributed devices' pattern
I was wanting to use, had a working CAN implementation on Arduino, and had very
nicely commented reference code that made it relatively easy to understand how
to write new behaviours.

## System description

The Arduino BCM System has the following design goals:

*  Battery operated, so energy efficient, using Arduino sleep modes and wake up.
*  No central logic unit - each Arduino contains code to handle its own outputs.
*  Entirely home built - no commercial devices or software needed.
*  Arduino - no Raspberry PI or other heavyweight OS needed

The system as implemented in my vehicle (Ford Econovan Maxi campervan) contains
a number of individual Arduino devices that perform distinct functions.  The
first thing you should do if implementing a system like this is to design how
you will be getting all your inputs and outputs hooked up.

### Main loom

The main loom unit is designed to simply measure all the inputs and control all
the outputs in the main wiring loom, and anything else nearby.  It is physically
situated by the steering column, so measures signals from the steering column
and stalk controls, ignition circuits, as well as signals coming from the body
of the van to the old dashboard gauge cluster.

Eventually this unit will also control the lights, windscreen wipers, washer
motor, and detect the reverse signal from the gearbox, door open switches,
interior lighting...  this one will need a lot of inputs and outputs!

### Gauge cluster

I built a custom gauge cluster using cheap gauges from Aliexpress and a Nextion
HMI 3.5" touchscreen display.  This Arduino grabs GPS data from a GPS module,
and outputs the data on the CAN bus to the gauges.  It also controls a backlight
for the gauges.

### Dashboard

Since I wanted the gauge cluster to be swappable with the original one so I
could keep driving the van while working on the cluster, I made a separate unit
to control the other functions of the dashboard - at this point just the stereo,
fan, and clock.

### Overhead

Since the rear of my vehicle has no window, I have no rearview mirror. Instead
I have a complete camera system with 5 cameras for rearview, reversing, left and
right blindspots, and front parking.  I also have a video screen where the
rearview mirror used to be, as well as a few switches for controlling various
bits and pieces like fog lights, marker lights and the like.

### Charge control

I have a starter battery and a house battery, plus a 90W solar panel mounted on
the roof.  Prior to installing this system, I had a 'Voltage Sensitive Relay'
installed between them, which connected the two batteries whenever either had a
voltage above ~13V and disconnected them whenever the voltage went below ~13V,
in other words if the alternator was putting out charge then both batteries
were in parallel and if the solar panel was putting out charge then both
batteries were also in parallel.  It died.

In reality, it's better to charge each battery separately.  I've no doubt that
my VSR died after I'd drained my house battery to 11.8V but the starter battery
was still charged up to 12.8V.  Sun hits the solar panel, VSR closes, and 1V
difference gets applied over the 0 resistance of the VSR - if you know your Ohms
Law, that equates to infinity amps...

So the charge control unit measures battery voltage of each battery and decides
when to switch the alternator or the solar panel to each battery.

### Stereo

My camper has a particularly complex stereo system.  The head unit has been
modified to output the on/off and volume knob settings via CAN bus, and the
audio is then sent to a rack of processors and amplifiers that need to be
powered up in a specific order.  A volume control signal is then applied to the
processor system.

This Arduino controls the power-up/down signals and outputting the volume
control voltage.

## Writing the code
