What is bukapilot?
------
[bukapilot](http://github.com/kommuai/bukapilot) is an open source software for advanced driver's assistant system (ADAS). It is a maintained patchset based on [openpilot](http://github.com/commaai/openpilot) which aims to support all Malaysian vehicles especially for Perodua and Proton by using hardwares specially designed for them, while adding complete right hand drive support on top of the existing capability from openpilot.

bukapilot performs the functions of Adaptive Cruise Control (ACC), Lane Keep Assist (LKA), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of supported car [makes, models and model years](http://kommu.ai). In addition, while bukapilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers.

Integration with Stock Features
------
All supported vehicles:
* Stock LKA is replaced by bukapilot LKA, which only functions when bukapilot is engaged by the user.
* Stock LDW is replaced by bukapilot LDW.

Additionally, on specific supported cars:
* Stock ACC is replaced by bukapilot ACC.
* bukapilot FCW operates in addition to stock FCW.

bukapilot should preserve all other vehicle's stock features, including, but are not limited to: FCW, Automatic Emergency Braking (AEB), auto high-beam, blind spot warning, and side collision warning.

Limitations of bukapilot
------

bukapilot do not automatically drive the vehicle or reduce the amount of attention that must be paid to operate your vehicle. The driver must always keep control of the steering wheel and be ready to correct the bukapilot LKA action at all times.

While changing lanes, bukapilot is not capable of looking next to you or checking your blind spot. Only nudge the wheel to initiate a lane change after you have confirmed it's safe to do so.

Many factors can impact the performance of bukapilot, causing them to be unable to function as intended. These include, but are not limited to:

* Very poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The camera is obstructed, covered or damaged.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The device is mounted incorrectly.
* When in sharp curves, like on-off ramps, intersections etc...; bukapilot is designed to be limited in the amount of steering torque it can produce.
* In the presence of restricted lanes or construction zones.
* When driving on highly banked roads or in presence of strong cross-wind.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Driving on hills, narrow, or winding roads.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of bukapilot components. It is the driver's responsibility to be in control of the vehicle at all times.

User Data and Kommu Account
------

By default, bukapilot uploads the driving data to our servers. You can also access your data by pairing with the KommuApp ([iOS](https://apps.apple.com/us/app/comma-connect/id1456551889), [Android](https://play.google.com/store/apps/details?id=ai.comma.connect&hl=en_US)). We use your data to train better models and improve bukapilot for everyone.

bukapilot logs the road facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver facing camera is logged by default but you can explicitly opt-out in settings. The microphone is not recorded.

By using bukapilot, you agree to [our Privacy Policy](https://kommu.ai/privacy/). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of Kommu. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to Kommu for the use of this data.

Safety and Testing
----
DISCLAIMER: bukapilot is still under development by a limited number of members. Duty of care will be taken to ensure the safety of both the hardware and the software. 

As a patchset of openpilot, bukapilot benefits from the software safety model of openpilot.  Incremental tests are being implemented to ensure the correctness of the patchset on top of the openpilot release.

Community and Contributing
------

bukapilot is developed by a local Malaysian team [Kommu](https://kommu.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/kommuai/bukapilot). Bug fixes and new car ports are encouraged.

And [follow us on Instagram](https://www.instagram.com/kommu.ai/).

Licensing
------

bukapilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless against Kommu Sdn Bhd. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

</img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
