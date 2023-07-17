# mujoco_contact_surfaces_plugin

Implementation of hydroelastic contact surfaces as a mujoco plugin.

## Install

```
mkdir build
cd build
make
cp libmujoco_contact_surfaces_plugin.so $MUJOCO_HOME/bin/mujoco_plugin
```

## Example

```
$MUJOCO_HOME/bin/simualte model/myrmex_surface_world.xml
```

The [sensor_config](https://github.com/fpatzelt/mujoco_contact_surfaces_plugin/blob/3289ff57d39bd9c55cad439804d4cddca31bc0a7/model/myrmex_surface_world.xml#L22C22-L22C35) must be given with an absolute path or relative to libmujoco_contact_surfaces_plugin.so.
