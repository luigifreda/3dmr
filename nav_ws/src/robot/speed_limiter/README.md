# speed\_limiter

This is a package for "limiting" `cmd_vel` commands according to speed limits that can be set by multiple independent 
"sources".

Each "source" publishes its own limit, and this node takes the minimum of all sources as the current limit. Note that
there is no method for "clearing" all currently active limits, so to allow full speed, you may need to cancel the 
limits from all "sources" separately.

Maximum linear speed is 0.6, maximum angular speed is 1.24 (values taken from `nifti_teleop`).

There is an OCU plugin for visualization called `speed_limiter_ocu_plugin` in `tradr-user-interaction/tradr_ocu`.

## Topics

- `cmd_vel_in` The input (desired) `cmd_vel`.
- `cmd_vel_out` The output (filtered, limited) `cmd_vel`.
- `adapt_trav_vel_in` All limit "sources" publish to this topic. The "sources" are differentiated based on the content 
  of `header.frame_id` of the messages they send (if it is empty, the name of the publisher is used instead).   
  Only `twist.linear.x` and `twist.angular.z` are used as the limits. 
  Pass `NaN` to these fields to cancel the limit from one "source". 
  When using `rostopic` from commandline, `NaN` is written as `.NaN`.
- `adapt_trav_vel` The currently active limit. `header.frame_id` contains a string `$linear;$angular`, where `$linear`
  is the name of the "source" that currently limits linear speed the most, and respectively for `$angular`.
  Both `$linear` and `$angular` can be empty strings if there is no limit. 
  
## Overriding

If `header.frame_id` is the special value `override`, it means that the speed limiter should be temporarily turned on (
  if `twist.linear.x` is a number) or off (if `twist.linear.x` is `NaN`).