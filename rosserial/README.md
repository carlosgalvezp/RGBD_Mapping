rosserial
=========

The Fuerte version of rosserial, despite claiming to support ROS services, did not actually support ROS services. I made the necessary changes to rosserial_python to support services. I then found that the whole thing has been rewritten (presumably for groovy) in the development repo at https://kforge.ros.org/rosserial/hg. The code there may or may not support services properly, but I suspect it does. In the mean time, anyone on fuerte who wants to can run this. For more info, see http://answers.ros.org/question/48548/rosserial-service-failed-to-parse-subscriber/