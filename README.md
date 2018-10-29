# cxq41_ps5_irb120_reactive

Code created by CXQ41@case.edu, the great and smart Frank Qian (Just kidding)...

Dependencies adopted from Dr. Neuman's 473 sample code...

## Running it...

To run this "fun" code, you will also need irb120 dependencies. I have created a minimal dependency so that you don't need to download everything from learning ros, but just the specific for running this code demo. Dependencies can be found here: (Credit to Dr. Neuman)
https://github.com/frank-qcd-qk/EECS473-IRB120_Dependencies.git

The dependencies should be put the same location with your code in roscd src folder.

If you have previously compiled irb120 code from learning ros before, it is highly recommended that you clean the build, devel and install folder so that there are no wired issue of package mis behavioring.

To see the path in a visualized way, I recommend to take a look at jsk's rvis add-on, link: 
http://wiki.ros.org/jsk_rviz_plugins

The included RVIZ config defaultly runs in the environment where jsk rviz plugin is installed.

## To start the code:

roslaunch cxq41_ps5_irb120_reactive cxq41_ps5.launch

rosrun cxq41_ps5_irb120_reactive cxq41_ps5_irb120_reactive

Some potential intial test poses for gear_part_ariac include: (-0.4,0.2), (-0.4,-0.2), (0.3,-0.2)

Be aware that robot has physical limitation and some position are just simply non-reachable...

## Live demo...
Find it here:
https://youtu.be/w5ESwQgUAdI

Chinese useres:
Link not yet available via Bilibili.
