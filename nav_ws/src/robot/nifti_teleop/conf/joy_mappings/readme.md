# Universal gamepad translator config files

The task of the universal joystick translator is to "translate" the joy_node outputs so that the button/axes indices 
are the same as for the referential Logitech RumblePad 2 gamepad.
  
This allows writing code that doesn't need to care about which particular type of joystick (in which mode) is plugged 
in. But since there are parts of code that expect different kinds of gamepads, it is still needed to provide a 
"backwards compatible" way.

The translator is implemented in `nifti_teleop/nodes/joy_translator.py` and uses configuration files from this folder.

The launch file `nifti_teleop/launch/nifti_joy.launch` has an argument called `transparent_translation`. If it is true,
the RumblePad-like data are published to the `joy` topic, and the "raw" (untranslated) data are published to topic 
`joy_raw`. This is the backwards-incompatible way. If `transparent_translation` is false, the `joy` topic contains the
 untranslated data, and topic `joy_translated` contains the translated data. This is the backwards-compatible way.
 
By default, `local_joy` and `any_joy` run with `transparent_translation` on, since they're only used by the low-level 
robot code that was never inteded to work with any other kind of joystick. All other joy topics (like `teleop_joy`) are 
run in the backwards-compatible way with `transparent_translation` off. So all old code that expected a specific joy 
type still works. But if the author decides to make it compatible with all gamepads, he just subscribes to 
`joy_translated` instead of `joy` (or to `any_joy` if it makes sense), adjusts the button/axis indices in his code, and 
the code is ready to work with any kind of supported gamepad.

## Adding a new gamepad model

Attach the gamepad to your computer. From `ls -l /dev/input/by-id/` find out the device name (let's call it `$DEV_NAME`).
If `$DEV_NAME` contains a serial number nerby the end of it, replace the serial number with string `SERIAL`. Then create
file named `$DEV_NAME.yaml` (best by copying `usb-Logitech_Logitech_Cordless_RumblePad_2-joystick.yaml`).

Now issue a `lsusb` in console and try to find the vendor:model pair for the gamepad. Create a symlink called 
`vendor:model.yaml` pointing to `$DEV_NAME.yaml`.

Next, edit one of the created YAML files. In one console run `rosrun joy joy_node`, in another one run 
`rostopic echo /joy`. Now observer the output of the `/joy` topic and try to match button/axis indices to the
functions in the YAML file. The filled-in values are passed to `eval()`, so you can use a bit of Python programming 
inside (e.g. math operations). 

While evaluating the values from the YAML file, some variables are accessible:

* `a`: a list of raw axis values
* `b`: a list of raw button values
* `pa`: a list of raw axis values in the previous message
* `pb`: a list of raw button values in the previous message
* `vars`: a dictionary of user-defined variables (you should initialize them in the `init` section)

In section `init`, you can add key-value pairs, where the key doesn't matter (it's just a nice name), and the value is 
passed to `eval()` during startup of the translator. This is nice for initializing user variables in the `vars` dict.

Section `callbacks` is similar to `init`, but the values are evaluated after each message is processed. You can e.g. 
change values in the `vars` dictionary from the callbacks. 

## TL;DR

If you haven't utilized `nifti_teleop` before (e.g. used the gamepad for anything else than steering the robot), you 
can ignore this readme. But you can also utilize the newly written `joy_translator` node that makes it easier to write 
code independent on a particular controller model. 

If your code subscribes to `local_joy/joy` or `any_joy` and expects any other joy type than RumblePad 2, you've got a
problem :) Otherwise, your code should work as before.

**From now on, you can attach any kind of joystick in the robot, and it works.**

All remote-control joy launchers that intend to steer the robot should include the 
`nifti_teleop/launch/nifti_joy.launch` launcher and should not edit copies of it. This ensures consistent behavior.