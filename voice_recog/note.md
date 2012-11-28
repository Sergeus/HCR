Voice Recognition notes
=======================

System architecture
-------------------

The system is split into three modules:

* Speech recogniser
* Speech Engine
* Text To Speech

The Speech Engine has a state machine to model a conversation. It can prompt the user by publishing a String to the Text To Seech node, and can get answers by subscribing to the speech recogniser.

Getting it working
------------------

Getting the whole system to work can be a bit fiddly. Here are a few notes and links which might help:

* __Speech Recognition__
  * The speech recogniser is based on pocketsphinx.
  * There is a pre-build ROS node for this, but it can be tricky to set up.
  * Can be ontained here: http://www.ros.org/wiki/pocketsphinx
  * Other help: http://bit.ly/To4Dht ; http://www.pirobot.org/blog/0022/ ; http://bit.ly/RiMRAS ; http://cmusphinx.sourceforge.net/wiki/gstreamer
  * You _might_ need to add a manifest in the `pocketsphinx` folder to get it working with ROS
  * I needed to install `gstreamer0.10-gconf` to get it working (http://bit.ly/VdZ7Ea)
  * You _may_ need to install `libasound` if it does now work. Test it first
  * May need to look out for directories ard coded to my setup (this will be fixed soon)

* __Speech Engine__
  * No real magic here, may need to look out for directories hard coded to my setup (this will be fixed soon)

* __Text To Speech__
  * Currently uses CMU flite (Might be a better alternative: need to look into this)
  * Can be obtained here: http://www.speech.cs.cmu.edu/flite/
 * Had difficulties getting it working as a c++ ROS node, so current system is a bit hacky in that it is a python node that calls `flite` through the command line


