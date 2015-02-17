Dissertation Project



Repo used for my City Univesity dissertation project: 

"Autonomous/user drone control with augmented reality implementation"


I developed a sea rescue inspired game with gameplay executed using 
a Parrot AR.Drone 2.0 drone (Parrot, 2014) and the use of OpenCV 
(OpenCV, 2014), an augmented reality open source library.

The outcome of the project is a user and/or autonomous drone control 
system with an augmented reality sea rescue inspired game implementation. 

The user controls the drone movement forwards and backwards (pitch control), 
left and right (roll control) and altitude. The autonomous drone control 
handles the boundaries, marked on the ground with patterns. The drone’s 
vertical camera visualises the patterns, analyses the coordinates of the 
pattern in relation to the camera view and reacts in accordance with the 
information so that if the user issues a command to roll left and the drone 
is over the left boundary, it cancels the user’s command and issues an 
autonomous command to roll right back to the centre of the play area. 
In the same manner, the drone also analyses the altitude by reading the 
ultrasound emitter sensor’s data.

The game implementation is a very basic game inspired by sea rescue missions. 
It involves the user to hover under a certain altitude over a certain patter, 
representing a person or a crate that is casted away, to pick him/it up, then 
repeat the same procedure over the centre maker representing a platform to 
drop the saved person/crate, accumulating points in the process.



-----------------------------------------------------------------
 CV Drone (= OpenCV + AR.Drone)
 Copyright (C) 2013 puku0x
 https://github.com/puku0x/cvdrone
-----------------------------------------------------------------

INTRODUCTION
  CV Drone is free software; you can redistribute it and/or
  modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file cvdrone-license-LGPL.txt.
   (2) The BSD-style license that is included with this library in
       the file cvdrone-license-BSD.txt.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
  cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.

HOW TO INSTALL
  Please unzip "cvdrone-master.zip" into an arbitrary directory.

HOW TO UNINSTALL
  Please delete the cvdrone folder.

BEFORE YOU BUILD
  You should install Visual Studio before you build CV Drone.
  CV Drone supports VC++2005/2008/2010/2012.
  To download VS, please see http://www.microsoft.com/visualstudio/eng/downloads .

HOW TO USE
  1. Open \build\vs20xx\test.sln
  2. Press F7 to build.
  3. Press F5 (or Ctrl+F5) to run.
  4. You can play around with OpenCV. Sample codes are in "src\samples".

FOR AR.DRONE 1.0 USERS
  Please update your AR.Drone's firmware to 1.11.5.

FOR AR.DRONE 2.0 USERS
  Please update your AR.Drone's firmware to 2.4.7.

FOR VS2010 USERS
  You can't build CV Drone by VS2010 after you installed VS2012.
  To build VS2010, you should uninstall ".Net Framework 4.5" and re-install "4.0".

LIBRARY DEPENDENCIES
  CV Drone uses following libraries.
  - OpenCV 2.4.6
    http://opencv.org/
  - FFmpeg 2.0
    http://www.ffmpeg.org/
  - stdint.h/inttypes.h for Microsoft Visual Studio r26
    https://code.google.com/p/msinttypes/
  - POSIX Threads for Win32 2.9.1
    http://www.sourceware.org/pthreads-win32/

  License files for each library can be found in the 'licenses' folder.

Thank you.
