# OpenCV-VR-Controllers

Long story short as the title says, this repository is a project that I was working for a year or two in my try of creating my virtual reality controller. Exploiting the Razer Hydra controllers driver, existing in Steam, and VRidge, which supports FreeTrack for head tracking and positioning, I kinda managed to achieve my goal. But due to hardware, knowledge and time limitations, I had to give the project up.

### What is implemented in this project
- OpenCV libraries to track the controller in space like PSMove do
- Sixense library to communicate with hydra drivers and give the orientation and position of the simulated controllers
- FreeTrack library to give the head position in space for VRidge
- In the Arduino 's part, I made a port of [existing source code](https://github.com/Razor-AHRS/razor-9dof-ahrs) of Razor 9DOF sensor for LSM9DS1 sensor (SparkFun 9DoF Sensor Stick)

### Fields of improvement
- The code can definitely be parallelized in many ways, for example, each thread works with one camera or one controller or can use CUDA cores to process images to track the controllers
- Better hardware, for example, a camera working with faster fps or better GPU
- Better UI with GUI
