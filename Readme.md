# Tinyraytracer

Example output:

[![Example result](https://thumbs.gfycat.com/LeafyAstonishingJoey-size_restricted.gif | width=100)](https://gfycat.com/fr/leafyastonishingjoey)

(Background credit: [P. Horálek](https://www.facebook.com/PetrHoralekPhotography) / ESO)

This repository is based on the tinyraytracer project https://github.com/ssloy/tinyraytracer/ 
(see also the homework branch)

The code is changed so the background image rotates. Stitching the images to get a GIF needs to be done 
with an external tool.
 
### Reproduction steps

* Grab a (preferably high resolution) 360° panorama image. I used a large JPEG that can be found [here](https://www.eso.org/public/images/potw2019a/).

* Place it in the project directory and name it `360.jpeg`. 
* By default the resolution of the output images is 720x480. You can change this by editing `width` and `height` variables in the `render` function 
in `tinyraytracer.cpp`.
* The code will generate 200 images by default, which means a full rotation takes around 6 secs, assuming a framerate of 30fps.
If you want to change it, change the variable `nImages` in `tinyraytracer.cpp`.
* Create a folder named `output/` in the project directory.
* Run the code after compiling it (see compilation steps). Now `output/` should contain the resulting images. 
* Generate a gif from your jpeg files: you can either use an online service like this [one](https://gifmaker.me/),
or use Photoshop following the steps [here](https://www.youtube.com/watch?v=ZLtbtmz2vKo).

### Compilation steps
```sh
git clone --recurse-submodules https://github.com/elrhaff/tinyraytracer
cd tinyraytracer
git submodule update --init
mkdir build
cd build
cmake ..  
make
```
