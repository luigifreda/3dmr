
Setup the standard folder storing your gazebo models:
```
cd ~
git clone https://github.com/osrf/gazebo_models 
mkdir .gazebo
mv ~/gazebo_models ~/.gazebo/models
```

You can also find some nice gazebo worlds here:
https://dev.px4.io/v1.11_noredirect/en/simulation/gazebo_worlds.html 
https://github.com/PX4/PX4-SITL_gazebo-classic 
In order to let gazebo find the models therein, you can export: 
`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:"<path-to-repo>/PX4-SITL_gazebo-classic/models"`