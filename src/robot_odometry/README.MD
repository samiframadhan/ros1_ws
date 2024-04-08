# What this package is about

This package contains source file for odometry calculation. It depends on published speed value from topic '/motor/left' and '/motor/right' which uses motor_msgs/motor custom message definition. The speed value is expected to published in m/s unit, and the resulting calculation is carried out at '/odom' topic.

Currently, it's configured to not publish TF transform since the transform would be published by its EKF counterpart.  However, if you planned to use this as tf publisher, you can make the change by changing its default value to True in the src file. TF publisher of the same tf data should only be from one source, so change everything that publishes the same tf data as this one to proceed with the change

The deadreckoning theory can be inferred from differential drive kinematics like [this one](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf) from columbia university