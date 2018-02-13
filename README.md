# ROS-Air-Hockey-Simulator
This repo contains the code for an airhockey simulator/game written in python.
The airhockeyfield is streamed to ROS on the topic '/airhockey/field/image'.

## Getting Started
You can follow the instructions below to deploy this software to your local machine.

### Prerequisites
Install dependencies with ´install_dependencies.sh´.

### Install
1. Clone this repo into your catkin_ws
2. Start a roscore
3. Give 'run.py' execution rights: ´$ sudo chmod +x run.py´
4. Run the simulator: ´$ rosrun ROS-Air-Hockey-Simulator run.py´
5. (Optional) - To visualize the airhockey field in ROS run: ´$ rosrun visualize_image_stream visualize.py /airhockey/field/image´

## Contributing
Please read [CONTRIBUTING.md](insert_link_here.html) for details on our code of conduct, and the process for submitting pull requests to us.

## License
This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/PXL-IT/NSEC/blob/master/LICENSE.md) file for details.

### Authors
* [Maarten Bloemen](https://github.com/MaartenBloemen)
