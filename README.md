# Northwestern Design Competition 2012

![Esteban](.//media/image1.jpeg)

## Overview

I was cleaning up my dropbox and stumbled upon all the files for Esteban, the robot Michael Chen and I built for the 2012 Northwestern Autonomous Robot Design Competetion. I thought github would be a better place to host the files, so here we are.

Since all the code was stored as snapshots (i.e. the [Final.doc](https://phdcomics.com/comics.php?f=1531) approach to version control), I turned each snapshot into a distinct commit and spoofed the dates using [git-redate](https://github.com/PotatoLabs/git-redate). 

## Related Links
- [El Viaje de Esteban](https://viajedeesteban.tumblr.com/) - a project blog chronicling the build
- [Match Video](https://www.youtube.com/watch?v=B897-lxEE_c)
- [Competition Rules](https://sites.google.com/site/dcnuinfo/previous-competition-rules/2012-competition-rules)
- [Mechatronics Lab Wiki](http://hades.mech.northwestern.edu/index.php/Northwestern_Design_Competition#DC2012)
- [Article](https://www.mccormick.northwestern.edu/news/articles/2012/05/2012-design-competition-winners.html)

## The Team

The team consisted of myself, a mechanical engineering junior, and
Michael Chen, a civil engineering junior. This was our second design
competition.

All aspects of the design were done together. This covers the initial
brainstorming of ideas, building mockups to see what worked and what
didn't, and all major mechanical, electrical, and software design
decisions. For example, deciding the dimensions of a part, discussing
what kind of circuit would accomplish a task best, and what sort of
approach an algorithm would take to tackle a problem were all at the
very least discussed. As a result, both of us have an intimate knowledge
of every aspect of the robot.

In actually building the robot, tasks were often split up to maximize
efficiency. However, we would often work on parts in tandem so that we
could still talk to each other about what we were doing. So while we
designed the chassis together, I was the one who actually created all of
the CAD files and used them to cut out the chassis on the laser cutter.
I also wrote all of the code that the robot ran on. However, Mike would
go in and tune different parts of the code that needed tuning. Mike also
implemented most of the electrical design.

The product of our work was the robot named Esteban. He bore two giant
googly eyes which served as the namesake for our team: "Googly Eyes."

## Strategy

A quick run-down of the competition revealed three key points that would
end up influencing our design above all others

-   The cubes were made out of a light foam which could not be detected
    easily from afar with simple sensors

-   Three times as many points were scored for cubes deposited in the
    bonus zone rather than the regular scoring zone

-   There are massive penalties for manipulating more than five cubes at
    a time

Early on we resigned ourselves to trying to find cubes using some sort
of efficient random algorithm rather than actually detect them. This was
based on the assumption that cubes would be moved around a lot during
the competition. Later on we realized that actually programming in
several possible hard coded paths was a more reliable option. We would
program paths meant to drive over the starting positions of 8 or so
cubes. However, after four cubes were actually picked up we would exit
the path and drive towards the scoring zone to deposit them.

Encoders were used for keeping track of Esteban's location. Because of
collisions and the moving scoring zone, we needed to reset our location
often. Resetting on the long axis was accomplished by driving over the
center line. Resetting after exiting the scoring zone was done by
hitting one of the side walls to reset angle, and then using either the
center line or a cube to reset overall position.

We couldn't detect cubes from afar, but a series of break beams were
used to determine when they entered our chassis. We took advantage of
the cube's light material by using two fans: one to blow them into a
tower and one to blow them out.

A laser attached to a servo was used to find the scoring zone, which was
covered in retroreflective tape. Esteban turns himself so that his back
is facing the scoring zone and then drives towards the center line, 6
feet away from the zone. The laser is then swept, and the "average"
angle of the laser for which the beam bounces back is found. Esteban
then turns this average angle and begins to drive towards the zone. The
same technique is repeated three feet from the zone and one and a half
feet from the zone. This helps adjust for the movement of the zone.

## Mechanical Design

### Parts list

-   3 acrylic sheets -- 0.118"x12"x24"

-   1 1.5" acrylic hinge

-   1 4" long 3/8" diameter acrylic rod

-   2 90mm wheels

-   2 4mm universal mounting hubs

-   2 ¾" metal ball casters

-   2 drive motors

-   2 small servos

-   1 large servo

-   1 laser

-   7 bright red LEDs

-   8 phototransistors

-   8 push buttons

-   2 capacitive encoders

-   1 60mm case fan

-   1 80mm case fan

-   4 7.4 V polymer lithium ion batteries

-   1 11.1 V polymer lithium ion battery

-   A pair of 40mm googly eyes

### Chassis CAD Drawing

![estebandwg.JPG](.//media/image2.jpeg)

### CAD -- Sheets for Laser Cutting

![1.png](.//media/image3.png)

![2.png](.//media/image4.png)

![3.png](.//media/image5.png)

### Locomotion Technique

The two drive motors provided to us were used to move Esteban. They were
placed towards the back. Two metal ball casters were placed near the
front for balance.

PID motion control based on encoder feedback was implemented to control
the motors. The hope was that this would allow us to set a speed that
both motors would adhere to with little error, allowing for Esteban to
drive in straight lines and turn accurately. Unfortunately, a number of
things prevented this from working well in practice. One was that the
wheels would actually lift off the ground while driving. This was fixed
by placing steel bars by the wheels to weight them down and keep them on
the ground. Another was the poor relationship between voltage and speed;
setting a constant voltage would result in a widely varied speed. This
meant that even when controlling for the voltage using motion control,
the resulting speed would still vary so much that Esteban ended up
veering left or right while driving.

![DSC04223.JPG](.//media/image6.jpeg)
![DSC04222.JPG](.//media/image7.jpeg)

Eight push buttons were used for collision detection; two at each
corner. The two at each corner are tied together so we only had
information about which corner was colliding with something.

![DSC04213.JPG](.//media/image8.jpeg)

### Interaction with the cube

The front of Esteban is shaped like the letter "V," wide at the front
and narrowing down to a channel slightly wider than the cubes. Once a
crate is detected, a 60mm case fan is lowered using an acrylic arm and
turned on. This blows the cube up onto a ramp and into Esteban's tower.

![DSC04209.JPG](.//media/image9.jpeg)
![DSC04208.JPG](.//media/image10.jpeg)

Entrance into the tower was controlled by a door attached to a servo.
The door was opened to let cubes in and closed to keep them in. Once
Esteban was in the scoring zone, he would drive to the back wall and
turn on the tower fan, which would blow all the cubes out.

![DSC04215.JPG](.//media/image11.jpeg)
![DSC04210.JPG](.//media/image12.jpeg)

To keep the cubes from shooting out too far and missing the bonus zone,
a "doggy door" was attached to the exit using an acrylic hinge. This
alters the cubes trajectory so they point downwards more than straight
out.

![DSC04202.JPG](.//media/image13.jpeg)

The tower was capable of holding up to three cubes. The fourth cube was
stored just outside the tower. The front fan would be turned on but the
tower door would be kept closed, causing the cube to be shot up until it
hit a thin acrylic barrier. The front fan would then be turned off and
rotated back into its resting position. Thanks to the fan arm's unique
shape, it actually holds the fourth cube in place. We can then blow it
into the tower at a later point.

![DSC04221.JPG](.//media/image14.jpeg)
![DSC04233.JPG](.//media/image15.jpeg)

## Electrical Design

### Actuator Controllers

One large servo was used to control the fan arm. One small servo was
used to control the tower door, and another was used to control the
laser.

### Sensors

Five breakbeam sensors were used for detecting cubes inside Esteban: one
at the very front of the chassis, one just in front of the channel (near
the middle of the chassis), and three in the tower. The breakbeam
sensors were simply a bright LED pointing at a phototransistor. To
control for ambient light, we would read the phototransistor twice in a
cycle: once with the LED on and once with it off. We would use the
difference between the two readings as our check; if it dipped to 0, we
knew a cube was there.

![DSC04218.JPG](.//media/image16.jpeg)

Two color sensors were used for determining the current color of the
board. They were simply an LED and phototransistor placed next to each
other, pointed at the ground. White reflected a lot, purple
significantly less, and black almost not at all. The two color sensors
were "pulsed" just as the breakbeams were to control for ambient light.
Heat shrink was used as an additional precaution.

![DSC04234.JPG](.//media/image17.jpeg)
![DSC04238.JPG](.//media/image18.jpeg)

A laser/phototransistor combo attached to a servo was used for finding
the scoring zone. It was blinked on and off in the same way the LEDs
were to control for ambient light.

![DSC04216.JPG](.//media/image19.jpeg)

The encoders used for motion control and location awareness were each
fed into their own dsPIC33. These were then connected to the main PIC32
using SPI communication.

![DSC04242.JPG](.//media/image20.jpeg)

All the sensors were connected to the PIC32 in the following way:

![pins.png](.//media/image21.png)

### Power Management

Rechargeable polymer-lithium ion batteries were used this year. All were
connected to voltage regulators so that the same voltage would be always
supplied to our components. One 7.4 V battery was used to power the
PIC32, one to power the logic, and two in series to power the fans. One
11.1V battery was used to power the motors.

### Breakout Boards

The two breakout boards were mounted to Esteban's sides.

![DSC04205.JPG](.//media/image22.jpeg)

![DSC04206.JPG](.//media/image23.jpeg)

## Programming

The program was split across four user written files + four provided
(NU32.h and .c, LCD.h and .c).

They were:
```c
- main.h     // initialization and interrupts
- data.h     // definition of constants, conversion formulas, and global variables
- library.h  // declaration of all functions
- library.c  // initialization of all functions
```

### Functions
```c
// Timer functions

void initCoreTimer(void); // initialize the core timer to interrupt every 1 ms

void initTimer4Interrupt(void); // turn on timer4 interrupt at 250 Hz

void initTimer5Interrupt(void); // turn on timer5 interrupt at 10 Hz

// Pin Initialization functions

void initChangeNotification(void); // enable pull up resistors for collision detection pins

void initDigitalOut(void); // enable all digital outs

void initAnalogInput(void); // read in all used analog in pins

void initOutputCompare(void); // turn on both motors and all three servos

// Encoder functions

void initEncoderSPI(void); // initialize SPI communication with the dsPICs at 8MHz

long getEncoder1(int reset); // get the current number of ticks of encoder1

long getEncoder2(int reset); // get the current number of ticks of encoder2

// Servo functions

void rotateFanArm(int servoPos); // rotate the fan arm to a desired position

void rotateTowerDoor(int servoPos); // rotate the tower door to a desired position

int sweepLaser(float feet); // sweep the laser and calculate the distance to the zone

// Fan functions

void blowCubeIn(); // blow a cube into the tower

void blowCubeUp(); // blow a fourth cube up outside of the tower for storage

void blowCubesOut(); // blow all cubes in the tower out

void blowCubeInAndOut(); // blow the fourth stored cube into the tower and straight out

void checkForCubes(); // while driving, stop if a cube is detected so it can be captured

// Driving Functions

void setMotorSpeed(int dutyCycle1, int dutyCycle2); // set the voltage for both motors

void reverseDirection(); // change the voltage appropriate to drive in reverse

void driveDistance(float inches, int mode); // drive forwards or backwards a certain distance

void turnAngle(int degrees); // turn a certain amount of degrees

void driveToCenter(); // drive until the center line (i.e. ground color changes)

void driveToZone(); // drive to center, sweep laser multiple times on the way to the scoring zone

void resetAngle(); // keep colliding and adjusting angle until perpendicular

void resetAngleOnWall(int wall); // collide with the wall until perpendicular

void resetAngleInZone(); // collide with the back wall until perpendicular

void exitAndResetOnRight(); // exit the scoring zone and reset angle on the right wall

void exitAndResetOnLeft(); // exit the scoring zone and reset angle on the left wall

// Sweep Patterns

void firstSweepPattern1(); // first cube in front, two on middle line, one on first line

void firstSweepPattern2(); // first cube in front, three on second line

void firstSweepPattern3(); // the four cubes on the diagonal from the starting position

void firstSweepPattern4(); // reset angle on wall at center line, then all on middle line

void firstSweepPattern5(); // the four closest cubes

void secondSweepPattern1(); // three on middle line, two on fourth line

void secondSweepPattern2(); // first cube upon resetting on right wall

void secondSweepPattern3(); // three on diagonal leading from the cube in secondSweepPattern2();

void randomSweepPattern(); // drive completely randomly

void wallToWallSweepPattern(); // sweep the entire field, wall to wall

// Stringing it all together

void firstSweep(); // firstSweepPattern4()+exitAndResetOnRight()

void secondSweep(); //secondSweepPattern2()+3()+ exitAndResetOnLeft()

void randomSweep(); // drive randomly searching for cubes
```
### main

#### main
```
{
    Call every initialization function
    while (1) {
        if (user button is pressed) {
            firstSweep();
            secondSweep();
        }
    }
}
```

#### Core Timer Interrupt (1000 Hz)
```
{
    increment the global time variable
}
```

#### Motion Control Interrupt (250 Hz)
```
{
    Read in encoder1 and encoder2 and perform PID control based on the driving state (driving straight or turning)
}
```

#### Sensor Interrupt (10 Hz)
```
{
    Turn on all the LEDs
    Read all the phototransistors
    Turn off all the LEDs
    Read all the phototransistors
    Calculate the difference between the ON and OFF values
    Determine the number of crates in the machine
    Determine the color being read by the color sensors
    Determine if any collision detectors are being tripped
}
```


## Results

### Midterm Milestone

For the midterm milestone, we had cut and assembled the first version of
Esteban. While testing with him we found that cubes would often get
stuck inside Esteban because they very easily rotate; we had to design
for 2 √3, not 2 √2. In order to make sure the cubes could traverse
Esteban's interior once we expanded relevant dimensions, we rather
recklessly tore him apart and taped him back together. Once we were
satisfied we began altering the design. In the meantime, "Frankenbot"
was used to clear the Midterm Milestone.

![IMG\_20120410\_221156.jpg](.//media/image24.jpeg)

### Pre-Competition Demonstration

Every single system but driving was up and running great for the
pre-competition demonstration. We still had issues getting the robot to
drive in a straight line reliably, but Esteban was at least turning
fairly accurately. What remained for the final week was fixing this
issue to the best of our ability, and actually coding in strategy.

### Competition Results

Getting the robot to drive straight accurately was an exercise in
frustration. We began to throw in a whole host of precisely tuned
constants into the PID motion control code, making something that was
once simple into something horrendously complex. Additionally, these
constants had to be retuned as our motors degraded. In the 24 hours
leading up to the competition, both of our motors ended up losing the
ability to drive backwards and had to be replaced. After these accidents
we decided to only tune what we thought were our two best paths for the
competition. We decided that consistency would probably pay off in the
competition more than diversity.

The decision ended up paying off, as we were seeded first and we won
first place. Esteban successfully executed his two paths each time.
Occasionally we would drift too left or right to pick up a cube we had
intended to pick up. One time a cube overshot the bonus zone and fell
out of the arena. But we always ended our rounds with at least six cubes
in the bonus zone. That was enough to carry us to victory.

## Takeaways

Thanks to the lessons learned from competing in last year's design
competition, we understood what needed to be done this year to build a
great robot. Esteban was very precisely engineered, and it shows. He
executed everything we meant for him to do exactly as we meant for him
to do it, with the exception for driving.

We meant to use some sort of feedback control for driving this year, and
our choice of PID simply did not cut it. Perhaps we would have gotten
better performance had we based the motion control on current control
rather than just encoder feedback. If there had been time, I would have
definitely explored more avenues of getting reliable driving.
