^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package septentrio_gnss_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.3 (2022-11-09)
------------------
* New Features
   * Twist output option
   * Example config files for GNSS and INS
   * Get leap seconds from receiver
   * Firmware check
   * VSM from odometry or twist ROS messages
   * Add receiver type in case INS is used in GNSS mode
   * Add publishing of base vector topics
* Improvements
   * Rework RTK corrections parameters and improve flexibility
* Fixes
   * /tf not being published without /localization
   * Twist covariance matrix of localization
   * Support 5 ms period for IMU explicitly

1.2.2 (2022-06-22)
------------------
* Fixes
   * Memory corruption under adverse conditions
* Commits
    * Merge pull request `#66 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/66>`_ from thomasemter/dev/next2
      Fix memory corruption
    * Fix parameter warnings
    * Reset buffer size to 16384
    * Update changelog
    * Fix memory corruption
    * Replace maps with unordered_maps
    * Overload timestamp function
    * Fix frame ids for INS msgs
    * Add define to avoid usage of deprecated header
    * Change readme on gps-msgs packet
    * Add info on user credentials
    * Fix spelling in readme
    * Merge remote-tracking branch 'upstream/ros2' into dev/next2
    * Add comment for heading from pose
    * Contributors: Thomas Emter, Tibor Dome

1.2.1 (2022-05-16)
------------------
* New Features
   * Add login credentials
   * Activate NTP server if use_gnss_time is set to true
* Improvements
   * Add NED option to localization
* Fixes
   * IMU orientation for ROS axis convention
* Commits
    * Merge pull request `#63 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/63>`_ from thomasemter/dev/next2
      Small fixes and additions
    * Merge pull request `#60 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/60>`_ from wep21/support-rolling
      fix: modify build error for rolling/humble
    * Revert change for deprecation warning in Humble
    * Change links to reflect ROS2
    * Amend readme regarding robot_localization
    * Fix compiler warnings for humble
    * Add more explanations for IMU orientation in ROS convention
    * Fix formatting in readme
    * Fix package name in readme
    * Update readme
    * Update changelog
    * Fix IMU orientation for ROS axis orientation
    * Activate NTP only if GNSS time is used
    * Add NED option to localization
    * Set NMEA header to GP
    * Update readme and changelog
    * Activate NTP server
    * Add credentials for access control
    * fix: modify build error for rolling/humble
    * Contributors: Daisuke Nishimatsu, Thomas Emter, Tibor Dome

1.2.0 (2022-04-27)
------------------
* New Features
   * Add option to use ROS axis orientations according to REP103
   * Add frame_id parameters
   * Add option to get frames from tf
   * Publishing of cartesian localization in UTM (topic and/or tf) for INS
   * Publishing of IMU topic for INS
   * Publishing of MeasEpoch
   * ROS2 branch
* Improvements
   * Add multi antenna option
   * Increase number of SBF streams
   * Add option to set polling_period to "on change"
   * Increased buffer size from 8192 to 131072 bytes
   * Add endianess aware parsers
   * Only publish topics set to true
   * Add parameter to switch DEBUG logging on and off
   * Change GPxxx messages to ROS built-in types
   * Remove duplicate INS msg types
* Fixes
   * Setting of antenna type
   * Publishing rate interconnections of gpsfix and velcovgeodetic
   * Missing quotes for antenna type
   * Broken attitude parsing pose and gpsfix from INS
   * IMU orientation was not sent to Rx
   * Graceful shutdown of threads
* Commits
    * Merge branch 'dev'
    * Prepare new release
    * Prepare new release
    * Merge pull request `#53 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/53>`_ from thomasemter/dev/refactor
      Very last changes
    * Add geographic lib dependency to package.xml
    * Add comment for frame of main antenna
    * Move utm zone locking section in readme
    * Reformulate readme section on frames
    * Merge pull request `#52 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/52>`_ from thomasemter/dev/refactor
      Last changes
    * Change frame id back to poi_frame_id
    * Make error log more explicit
    * Merge pull request `#49 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/49>`_ from thomasemter/dev/refactor
      Improve IMU blocks sync and do-not-use value handling
    * Fix buffer size in changelog
    * Turn off Nagle's algorithm for TCP
    * Fix changelog formatting
    * Fix readme
    * Set default base frame to base_link
    * Fix valid tow check logic
    * Increase buffer size for extreme stress tests
    * Fix crc check
    * Fix and streamline tf handling
    * Add checks for validity of values
    * Fix rad vs deg
    * Update changelog
    * Add some comments
    * Set stdDevMask to values > 0.0 in node
    * Set stdDevMask to values > 0.0
    * Add info on RNDIS and set it to default
    * Increase default serial baud rate
    * Add parameter to set log level to debug
    * Change defaults for publishers in node
    * Put publish params together and fix mismatch in readme
    * Improve IMU blocks sync and do-not-use value handling
    * Merge pull request `#48 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/48>`_ from thomasemter/dev/refactor
      Fix measepoch not publishing without gpsfix
    * Fix measepoch not publishing without gpsfix
    * Merge pull request `#47 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/47>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Publish only messages set to true
    * Remove leftover declaration
    * Merge branch 'dev/endianess_agnostic' into dev/refactor
    * Update readme to reflect endianess aware parsing
    * Remove msg smart pointers
    * Fix array assertion failure
    * Cleanup
    * Add ReceiverStatus parser
    * Add QualityInd parser
    * Add DOP parser
    * Add ReceiverSetup parser
    * Fix MeasEpoch and ChannelStatus parsers, add measepoch publishing
    * Add ChannelStatus parser
    * Add MeasEpoch parser
    * Add IMU and VelSensor setup parsers
    * Add Cov SBF parsers
    * Add templated qi parser function
    * Add AttEuler+Cov parser
    * Revert ordering change inside INSNav ROS msgs
    * Add ExtSensorMeas parser
    * Add PVT parsers
    * Add range checks to parsers
    * Replace INSNav grammar with parsers
    * Test parser vs. grammar for better performance
    * Fix sb_list check
    * Add IMU and VelSensor setup grammars
    * Move adapt ROS header to typedefs.h
    * Add revision check to MeasEpoch
    * Fix ReceiverStatus grammar
    * Extend ReceiverSetup and add revision checks
    * Change logger and fix loop range
    * Remove reserved bytes from parsing
    * Remove obsolete structs
    * Directly parse Cov SBFs to ROS msg
    * Directly parse PVT SBFs, remove obsolete ids
    * Rename rev to revision
    * Fix block header parsing
    * Directly parse AttEuler to ROS msg
    * Directly parse to ROS msgs for INSNavXxx
    * Exchange pow with square function and remove casts
    * Merge pull request `#46 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/46>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Simplify sync bytes check
    * Move tow/wnc to BlockHeader
    * Adjust order in INSNav ros msgs
    * Fix INSNav grammars
    * Change BlockHeader structure
    * Remove length ref from header
    * Rectify sb_list check of INSNavXxx
    * Add automtatic activation of multi-antenna mode
    * Merge branch 'dev/refactor' of https://github.com/thomasemter/septentrio_gnss_driver into dev/refactor
    * Add automtatic activation of multi-antenna mode
    * Fix wrong scope of phoenix::ref variables
    * Fix AttEuler grammar
    * Add max size checks to QualityInd and ReceiverStatus
    * Replace locals with phoenix::ref in grammars
    * Add revision dependent parsing to PVTs
    * Change offset check to epsilon
    * Change offset check to epsilon
    * Fix parsing checks
    * Set has arrived to false on parsing error
    * Add INSNav grammars
    * Add abs to offset check
    * Add abs to offset check
    * Add Cov grammars
    * Remove superfluous typdefs of structs
    * Add ReceiverStatus grammar
    * Add QualityINd grammar
    * Merge pull request `#45 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/45>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Add id check to header grammar
    * Add id check to header grammar
    * Add ReceiverSetup grammar
    * Add DOP grammar
    * Directly intialize vector to parse
    * Add MeasEpoch grammar
    * Remove duplicate msg types
    * Remove obsolete include
    * Add revision and length return to header grammar
    * Merge branch 'feature/endianess_agnostic' into dev/endianess_agnostic
    * Make multi_antenna option also usable for gnss
    * Add typedefs plus some minor changes
    * Add warning concerning pitch angle if antennas are rotated
    * Add multi antenna option to ins and fix antenna offset decimal places trimming
    * Fix identation
    * Distinguish between gnss and ins for spatial config from tf
    * Merge pull request `#43 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/43>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Add vehicle frame for clarity
    * Handle missing tf more gently
    * Merge branch 'dev/spatial_config_via_tf' into dev/refactor
    * Update readme
    * Fix antenna offset from tf
    * Add automatic publishing of localization if tf is activated
    * Add automatic publishing of localization if tf is activated
    * Add spatial config via tf, to be tested
    * Fix crashes due to parsing errors (replacing uncatched throws)
    * Add tf broadcasting
    * Add comments
    * Add localization in UTM output
    * Add check to IMU msg sync
    * Change msg sync to allow for 200 Hz IMU msgs
    * Add ROS IMU msg
    * Fix IMU setup message attitude conversion
    * Fix pose from INS data
    * Fix IMU raw data rotation compensation
    * Make antenna attitude offset usable by GNSS
    * Add ros directions option to pose and fix covariances
    * Update readme
    * Merge branch 'feature/ros_axis_orientation' into dev/refactor
    * Add nmea_msgs dependencies
    * Merge branch 'dev/nmea' into dev/refactor
    * Update readme
    * Update readme
    * Add antenna offsets to conversions
    * Fix IMU orientation conversion
    * Change ExtSensorMead temperature to deg C
    * Add axis orientation info to readme
    * Fix IMU axis orientation
    * Change get int param
    * Update readme to reflect removal of aux antenna offset
    * Fix different antenna setup message for INS and remove obsolete aux1 antenna offset for GNSS
    * Fix ExtSensorMeas message filling
    * Fix ExtSensorMeas message to reflect available fields
    * Fix missing INS blocks
    * Fix missing INS blocks
    * WIP, introduce ros axis orientation option, to be tested
    * Add option to set pvt rate to OnChange
    * Add comment on NTP to readme
    * Change to nmea_msgs
    * Add automatic addition of needed sub messages
    * Comment out setting debug level
    * Add comments and fix spelling errors
    * Merge pull request `#42 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/42>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Change to quaternion msg typedef
    * Comment out debug logging
    * Remove filling of seq field
    * Change msg definitions to be compatible with ROS2
    * Update readme
    * Change make_shared for portability and add more typedefs
    * Add get param int fallback for numeric antenna serial numbers
    * Change Attitude to be published with pvt rate
    * Add log identifier
    * Add checks for relevant ros params
    * Concatenate multiple SBF blocks in streams
    * Move main into own file
    * Move get ros time to AsyncManager
    * Remove obsolete param comment
    * Move get ros params to base class
    * Change to nsec timestamp internally
    * Add publishing functionality to node base class
    * Move node handle ptr and functions to base class and rename
    * Add stamp to nmea parsing
    * Add logging in PcapReader
    * Add logging in CircularBuffer
    * Add missed logging
    * Add logging in AsyncManager
    * Add getTime function
    * Add logging in RxMessage
    * Add logging in CallbackHandlers
    * Add log function to node by polymorphism, logging in Comm_OI
    * Fix wait function and force use_gnss_time when reading from file
    * Add thread shutdown and remove spurious delete
    * Add typedefs for ins messages
    * Add typedefs for gnss messages
    * Add typedefs for ros messages
    * Refine shutdown
    * Fix shutdown escalating to SIGTERM
    * Move waiting for response in send function
    * Make functions private
    * Change crc to C++
    * Fix variable name
    * Remove global variables from node cpp file
    * Move more global settings to settings struct
    * Move more global settings to settings struct
    * Move global settings to settings struct
    * Move more functions to Comm_IO
    * Move settings to struct and configuration to Comm_IO
    * Merge branch 'dev/change_utc_calculation' into dev/refactor
    * Remove obsolete global variables
    * Move g_unix_time to class
    * Make has_arrived booleans class memebers and rx_message a persistent class
    * Make node handle a class member
    * Fix parsing of ID and rev
    * Finish ChannelStatusGrammar, to be tested
    * WIP, partially fix ChannelStatusGrammar
    * Add SBF length parsing utility
    * Insert spirit parsers
    * WIP, add omission of padding bytes
    * WIP, add more spirit parsers
    * Add parsing utilities for tow, wnc and ID
    * Move getId/Tow/Wnc to parsing utilities
    * Change UTC calculation to use tow and wnc
    * WIP, add boost spirit and endian buffers
    * Change UTC calculation to use tow and wnc
* ROS2 Commits
    * Prepare ros2 release
    * Merge pull request `#54 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/54>`_ from thomasemter/dev/ros2
      ROS2 branch
    * Port driver to ros2
* Change UTC calculation to use tow and wnc
* Contributors: Thomas Emter, Tibor Dome, tibordome

1.0.8 (2021-10-23)
------------------
* Added INS Support

1.0.7 (2021-05-18)
------------------
* Clang formatting, publishing from SBF log, play-back of PCAP files

1.0.6 (2020-10-16)
------------------
* ROSaic binary installation now available on Melodic & Noetic

1.0.5 (2020-10-15)
------------------
* changed repo name
* v1.0.4
* 1.0.3
* Merge pull request `#22 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/22>`_ from septentrio-gnss/local_tibor
  New changelog
* New changelog
* Merge pull request `#21 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/21>`_ from septentrio-gnss/local_tibor
  Added rosdoc.yaml file
* Merge pull request `#20 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/20>`_ from septentrio-gnss/local_tibor
  Improved doxygen annotations
* Merge pull request `#19 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/19>`_ from septentrio-gnss/local_tibor
  Improved doxygen annotations
* Update README.md
* Merge pull request `#18 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/18>`_ from septentrio-gnss/local_tibor
  Adopted ROS and C++ conventions, added ROS diagnostics msg,
* Update README.md
* Update README.md
* Update README.md
* Contributors: septentrio-users, tibordome

1.0.4 (2020-10-11)
------------------
* Added rosdoc.yaml file
* Improved doxygen annotations
* Improved doxygen annotations
* Adopted ROS and C++ conventions, added ROS diagnostics msg, removed ROS garbage value bug, added auto-detection of SBF arrival order for composite ROS msgs
* Merge branch 'master' of https://github.com/septentrio-gnss/rosaic
* NTRIP with Datalink, circular buffer, reading connection descriptor, new messages
* Update README.md
* Contributors: septentrio-users, tibordome

1.0.3 (2020-09-30)
------------------
* Add new config/rover.yaml file
* Add config/rover.yaml to .gitignore
* Merge pull request `#17 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/17>`_ from septentrio-gnss/local_tibor
  NTRIP with Datalink, circular buffer, reading connection descriptor..
* Merge branch 'local_tibor'
* NTRIP with Datalink, circular buffer, reading connection descriptor, new messages
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#16 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/16>`_ from septentrio-gnss/local_tibor
  NTRIP parameters added, reconnect_delay_s implemented,
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#15 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/15>`_ from tibordome/local_tibor
  GPSFix completed, datum as new parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#14 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/14>`_ from tibordome/local_tibor
  GPSFix completed, datum as new parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#13 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/13>`_ from tibordome/local_tibor
  Added AttCovEuler.msg and AttEuler.msg
* Merge pull request `#12 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/12>`_ from tibordome/local_tibor
  Fixed service field of NavSatStatus
* Contributors: Tibor Dome, septentrio-users, tibordome

1.0.2 (2020-09-25)
------------------
* NTRIP parameters added, reconnect_delay_s implemented, package.xml updated, ROSaic now detects connection descriptor automatically, mosaic serial port parameter added
* GPSFix completed, datum as new parameter, ANT type and marker-to-arp distances as new parameters, BlockLength() method corrected, sending multiple commands to Rx corrected by means of mutex
* Contributors: tibordome

1.0.1 (2020-09-22)
------------------
* GPSFix completed, datum as new parameter, ANT type and marker-to-arp distances as new parameters, BlockLength() method corrected, sending multiple commands to Rx corrected by means of mutex
* Added AttCovEuler.msg and AttEuler.msg
* Fixed service field of NavSatStatus, fixed ROS header's seq field of each published ROS message, added write method for sending commands to Rx, successfully tested, added AttEuler, added AttCovEuler
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#11 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/11>`_ from tibordome/local_tibor
  rosconsole_backend_interface dependency not needed
* rosconsole_backend_interface dependency not needed
* Merge pull request `#10 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/10>`_ from tibordome/local_tibor
  rosconsole_log4cxx dep not needed
* rosconsole_log4cxx dep not needed
* Merge pull request `#9 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/9>`_ from tibordome/local_tibor
  rosconsole_log4cxx dep not needed
* rosconsole_log4cxx dep not needed
* Merge pull request `#8 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/8>`_ from tibordome/local_tibor
  Local tibor
* Update README.md
* Merge pull request `#7 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/7>`_ from tibordome/local_tibor
  Ready for First Release
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#6 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/6>`_ from tibordome/local_tibor
  Local tibor
* Merge pull request `#5 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/5>`_ from tibordome/local_tibor
  TCP seems to work
* Contributors: Tibor Dome, tibordome

1.0.0 (2020-09-11)
------------------
* Ready for first release
* Added Gpgga.msg and PosCovGeodetic.msg files
* Ready for First Release
* Ready for first release
* Ready for first release
* Ready for first release
* TCP bug removed
* TCP bug removed
* TCP seems to work
* Merge pull request `#4 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/4>`_ from tibordome/v0.2
  V0.2
* PVTCartesian and PVTGeodetic publishing works on serial
* PVTCartesian and PVTGeodetic publishing works on serial
* Merge pull request `#3 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/3>`_ from tibordome/v0.2
  Add doxygen_out and Doxyfile 2nd trial
* Add doxygen_out and Doxyfile 2nd trial
* Merge pull request `#2 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/2>`_ from tibordome/v0.1
  Add doxygen_out and Doxyfile
* Add doxygen_out and Doxyfile
* Update README.md
* Create README.md
* Update LICENSE
* Merge pull request `#1 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/1>`_ from tibordome/add-license-1
  Create LICENSE
* Create LICENSE
* Create LICENSE
* Commit
* Successfully tested publishing to /gpgga topic via serial
* To make sure master branch exists
* Contributors: Tibor Dome, tibordome

