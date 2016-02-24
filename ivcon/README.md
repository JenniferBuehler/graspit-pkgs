# Conversion to and from Inventor files

Inventor conversion library required for urdf2graspit.

This source code is heavily based on the **ivcon** package (original source which this is based on is provided in *ivcon.zip*).

It is *not* the same source as maintained in the [ROS ivcon package](http://wiki.ros.org/ivcon) git repository. Though this package
also refers to external code at [https://sourceforge.net/projects/ivcon/](https://sourceforge.net/projects/ivcon/), which is the code that is used here. 

Mainly to be noted is that I added a SCALE\_FACTOR (static variable in class IVCONV) which can be set in order to scale the whole mesh before writing it.
I also had to rewrite some other code in it in order to get it to work properly.

## Required system packages

- SoQt4 (ubuntu package *libsoqt4-dev*)
- Coin (ubuntu package *libcoin80-dev*)
- Qt4 (ubuntu package *libqt4-dev*)

## Important License note
 
This package contains minor modifications to the source in the "IVCON" 
package on sourceforge, [http://sourceforge.net/projects/ivcon/](http://sourceforge.net/projects/ivcon/).

The original code is licensed under the [Common Public License 1.0](http://sourceforge.net/directory/license:ibmcpl/)
See the full license code in [LICENSE](LICENSE).
