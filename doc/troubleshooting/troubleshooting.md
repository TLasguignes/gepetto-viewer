<link rel="stylesheet" href="qrc:/css/troubleshooting"/>

[gepetto-viewer]:https://github.com/gepetto/gepetto-viewer
[gepetto-viewer-corba]:https://github.com/gepetto/gepetto-viewer-corba

# Troubleshooting

## Summary

  * [Menus or tooltips are unreadable](#menus-or-tooltips-are-unreadable)
  * [Display of large meshes is very slow](#display-of-large-meshes-is-very-slow)
  * [`CORBA::TRANSIENT` when launching a server](#corbatransient-when-launching-a-server)
  * [Collada files are not displayed](#collada-files-are-not-displayed)
  * [I still have an issue](#i-still-have-an-issue)

---

## Menus or tooltips are unreadable

You can create a Qt style sheet `stylesheet.txt`:
```
QMenu {
 background: white;
 color: black;
}
QMenu:selected {
 background: blue;
 color: white;
}
QToolTip {
 background: yellow;
 border: 1px solid black;
 color: black;
}
```
and run `gepetto-gui -stylesheet stylesheet.txt`

[top](#troubleshooting)

---

## Display of large meshes is very slow

A solution to help reducing the rendering time is to use Level Of Details (LOD).
In short, you can use the following command to generate a LOD for mesh named `mesh.ext`.
```sh
gvtools --input mesh.ext --simplify 1,0,1 --simplify 0.5,1,3 --simplify 0.2,3,100 --output mesh.ext.osgb
```
This will create a LOD with three levels:
- from 0 to 1 meters, use original mesh,
- from 1 to 3 meters, use simplified mesh with ratio 0.5 (half less vertices),
- from 3 to 100 meters, use simplified mesh with ratio 0.2,
- farther than 100 meters, don't show anything.

`gvtools` is part of *gepetto-viewer* package.  See `gvtools --help` for more details.

[top](#troubleshooting)

---

## `CORBA::TRANSIENT` when launching a server

It very often happens that the OmniNames server failed to start properly at boot.

To check if the server is running, run:
```bash
ps -C omniNames -o pid,args
```

If the process is not running, delete omniNames related log and backup files in `/var/lib/omniorb`. They may have different names on your computer, but most likely, something like:
```bash
sudo rm /var/lib/omniORB/omninames-`hostname`.log
sudo rm /var/lib/omniORB/omninames-`hostname`.bak
```
then restart the server:
```bash
sudo service omniorb4-nameserver restart
```

[top](#troubleshooting)

---

## Collada files are not displayed

The nodes are created and exists in the body tree widget but nothing appears in the scene viewer. This is due to a conflict between OSG and Qt. A work around is to convert the DAE to osg using the following command:
```bash
# <file> is the name of the file including the DAE extension.
# The output filename will end with .dae.osg.
osgconv <file> <file>.osg
```
If you have many files, you may run that at the root of the meshes subdirectories:
```python
find ${root_of_meshes_directory} -iname "*.dae" -type f -exec osgconv {} {}.osg \;
```

[top](#troubleshooting)

---

## I still have an issue

Have a look at the issues already posted on [gepetto-viewer] and [gepetto-viewer-corba].

[top](#troubleshooting)
