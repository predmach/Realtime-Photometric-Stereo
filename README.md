Realtime-Photometric-Stereo
===========================

Implementing realtime photometric-stereo using a monochromatic Point Grey Chameleon camera and 8 leds. The camera runs at 15 frames/s with a resolution of 640x480. One reconstruction takes about 100 ms. 

[Live Demo on YouTube](http://www.youtube.com/watch?v=2JrwRT9_vO4)

![GUI Screenshot](https://raw.github.com/NewProggie/Realtime-Photometric-Stereo/master/assets/Screenshot-Realtime-PS.png


In case of the following error after running ./main  

terminate called after throwing an instance of 'boost::wrapexcept<boost::system::system_error>'
  what():  open: No such file or directory
Aborted (core dumped)

sudo chmod a+rw /dev/ttyACM0. 
