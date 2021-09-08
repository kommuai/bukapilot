What is bukapilot?
------

[bukapilot](http://github.com/kommuai/bukapilot) is a fork of an open source driver assistance system from [openpilot](http://github.com/commaai/openpilot). Currently, bukapilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of supported [car makes, models and model years](#supported-cars). In addition, while bukapilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers.

Integration with Stock Features
------
All supported vehicles:
* Stock Lane Keep Assist (LKA) and stock ALC are replaced by bukapilot ALC, which only functions when bukapilot is engaged by the user.
* Stock LDW is replaced by bukapilot LDW.

Additionally, on specific supported cars (see ACC column in [supported cars](#supported-cars)):
* Stock ACC is replaced by bukapilot ACC.
* bukapilot FCW operates in addition to stock FCW.

bukapilot should preserve all other vehicle's stock features, including, but are not limited to: FCW, Automatic Emergency Braking (AEB), auto high-beam, blind spot warning, and side collision warning.

We have detailed instructions for [how to install the device in a car](https://comma.ai/setup).

| Make      | Model                         | Supported Package | LKAS          | Gas            | Brake         |
| ----------| ------------------------------| ------------------| --------------| ---------------| --------------|
| Perodua   | Axia Advanced 2019            | ASA2.0            | KommuActuator | KommuActuator  | None          |


KommuAssist Installation Instructions
------

Follow these [video instructions](https://youtu.be/FktYvHD1PD0) to properly mount the device on the windshield. Note: bukapilot features an automatic pose calibration routine and bukapilot performance should not be affected by small pitch and yaw misalignments caused by imprecise device mounting.


You will be able to engage bukapilot after reviewing the onboarding screens and finishing the calibration procedure.

Limitations of bukapilot ALC and LDW
------

bukapilot ALC and bukapilot LDW do not automatically drive the vehicle or reduce the amount of attention that must be paid to operate your vehicle. The driver must always keep control of the steering wheel and be ready to correct the bukapilot ALC action at all times.

While changing lanes, bukapilot is not capable of looking next to you or checking your blind spot. Only nudge the wheel to initiate a lane change after you have confirmed it's safe to do so.

Many factors can impact the performance of bukapilot ALC and bukapilot LDW, causing them to be unable to function as intended. These include, but are not limited to:

* Poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The road facing camera is obstructed, covered or damaged by mud, ice, snow, etc.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The device is mounted incorrectly.
* When in sharp curves, like on-off ramps, intersections etc...; bukapilot is designed to be limited in the amount of steering torque it can produce.
* In the presence of restricted lanes or construction zones.
* When driving on highly banked roads or in presence of strong cross-wind.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Driving on hills, narrow, or winding roads.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of bukapilot components. It is the driver's responsibility to be in control of the vehicle at all times.

Limitations of bukapilot ACC and FCW
------

bukapilot ACC and bukapilot FCW are not systems that allow careless or inattentive driving. It is still necessary for the driver to pay close attention to the vehicle’s surroundings and to be ready to re-take control of the gas and the brake at all times.

Many factors can impact the performance of bukapilot ACC and bukapilot FCW, causing them to be unable to function as intended. These include, but are not limited to:

* Poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The road facing camera or radar are obstructed, covered, or damaged by mud, ice, snow, etc.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The device is mounted incorrectly.
* Approaching a toll booth, a bridge or a large metal plate.
* When driving on roads with pedestrians, cyclists, etc...
* In presence of traffic signs or stop lights, which are not detected by bukapilot at this time.
* When the posted speed limit is below the user selected set speed. bukapilot does not detect speed limits at this time.
* In presence of vehicles in the same lane that are not moving.
* When abrupt braking maneuvers are required. bukapilot is designed to be limited in the amount of deceleration and acceleration that it can produce.
* When surrounding vehicles perform close cut-ins from neighbor lanes.
* Driving on hills, narrow, or winding roads.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Interference from other equipment that generates radar waves.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of bukapilot components. It is the driver's responsibility to be in control of the vehicle at all times.

Limitations of bukapilot DM
------

bukapilot DM should not be considered an exact measurement of the alertness of the driver.

Many factors can impact the performance of bukapilot DM, causing it to be unable to function as intended. These include, but are not limited to:

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs/).

The list above does not represent an exhaustive list of situations that may interfere with proper operation of bukapilot components. A driver should not rely on bukapilot DM to assess their level of attention.

User Data and comma Account
------

By default, bukapilot uploads the driving data to our servers. You can also access your data by pairing with the Kommu app ([iOS](https://apps.apple.com/us/app/comma-connect/id1456551889), [Android](https://play.google.com/store/apps/details?id=ai.comma.connect&hl=en_US)). We use your data to train better models and improve bukapilot for everyone.

bukapilot logs the road facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver facing camera is logged by default but you can explicitly opt-out in settings. The microphone is not recorded.

By using bukapilot, you agree to [our Privacy Policy](https://kommu.ai/privacy/). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of Kommu. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to Kommu for the use of this data.

Safety and Testing
----
DISCLAIMER: bukapilot is still under development by a limited number of members. Duty of care will be taken to ensure the safety of both the hardware and the software.

* bukapilot observes ISO26262 guidelines, see [SAFETY.md](SAFETY.md) for more detail.
* bukapilot has software in the loop [tests](.github/workflows/test.yaml) that run on every commit.
* The safety model code lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* KommuSafety has software in the loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware in the loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware in the loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest bukapilot in a testing closet containing 10 EONs continuously replaying routes.

Community and Contributing
------

bukapilot is developed by a local Malaysian team [Kommu](https://kommu.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/kommuai/bukapilot). Bug fixes and new car ports are encouraged.

And [follow us on Instagram](https://www.instagram.com/kommu.ai/).

Directory Structure
------
    .
    ├── cereal              # The messaging spec and libs used for all logs
    ├── common              # Library like functionality we've developed here
    ├── installer/updater   # Manages auto-updates of bukapilot
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN
    ├── third_party         # External libraries
    ├── pyextra             # Extra python packages
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts, images, and sounds for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── camerad         # Driver to capture images from the camera sensors
        ├── car             # Car specific code to read states and control actuators
        ├── common          # Shared C/C++ code for the daemons
        ├── controls        # Planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Precise localization and vehicle parameter estimation
        ├── logcatd         # Android logcat as a service
        ├── loggerd         # Logger and uploader of car data
        ├── modeld          # Driving and monitoring model runners
        ├── proclogd        # Logs information from proc
        ├── sensord         # IMU interface code
        ├── test            # Unit tests, system tests, and a car simulator
        └── ui              # The UI

Licensing
------

bukapilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

</img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
