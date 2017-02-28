# ProjectionClock messages are used to synchronously advance the clock
# of a set of nodes that provide distributed projection capabilities.
# The original intention was to have it published by a central projection
# management node that acts as a sort of pace-maker for the projection.

time now         # time equivalent to ros::Time::now(), used to stamp data
duration period  # duration between clock signals, used to advance projections
