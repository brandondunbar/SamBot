# SamBot
The logic driving the robot.

<div id="top"></div>
<!--
*** README template src: https://github.com/othneildrew/Best-README-Template
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![GPLv3][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/brandondunbar/SamBot">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">SamBot</h3>

  <p align="center">
    The software for Small Area Mapping Bot (SamBot). Eventually will map out rooms it is placed in, and some day will bring shoes.
    <br />
    <a href="https://github.com/brandondunbar/SamBot/wiki"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://youtube.com/shorts/7y07qh7_wHk">View Demo</a>
    ·
    <a href="https://github.com/brandondunbar/SamBot/issues">Report Bug</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

<!--
[![Product Name Screen Shot][product-screenshot]](https://example.com)
-->
This project is intended to provide the developer(s) with an introduction to robotics. The challenge of mapping out a room implements SLAM, basic target setting and navigation, sensor fusion (if using multiple sensors), object avoidance, path planning, and other software concepts, not to mention the mechanical and electrical engineering topics introduced by building the platform it is running on.
The core of the robot is an Nvidia Jetson Nano; sensors are RPLidar A1 and Intel Realsense D435i; and output is implemented through GPIO to a motor driver board and two bi-directional wheels.
<p align="right">(<a href="#top">back to top</a>)</p>



### Built With

#### Software

* [Python3](https://www.python.org/)
* [Ros2 Foxy](https://docs.ros.org/en/foxy)
<!-- C++ -->

#### Hardware

* [Nvidia Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) - Primary Compute
* [RPLidar A1](https://www.slamtec.com/en/Lidar/A1) - Single-band, top-mounted LiDAR 
* [Intel Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) - A stereoscopic camera with an IMU
* [MBot](https://www.makeblock.com/steam-kits/mbot) - For the Chassis
* [CAD Files](https://cad.onshape.com/documents/557d8581beaba091eb3068c6/w/fc48db46dff306e9c886c59b5/e/04e9e5f2b46c381649233ec1) - 3D print ready

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This project isn't ready for distribution; so it must be built from source. Once ready, an image will be available.

### Prerequisites

* [Python3.6](https://www.python.org/downloads/) or greater
* [Ros2 Foxy](https://docs.ros.org/en/foxy/Installation.html) 
* [Colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/brandondunbar/SamBot.git
   ```
2. Navigate to SamBot/SamBot
   ```sh
   cd ./SamBot/Sambot
   ```
3. Create a virtual environment
   ```sh
   python3 -m venv venv
   ```
4. Activate it
   ```sh
   source venv/bin/activate
   ```
5. Install the required libraries
  ```sh
  python3 -m pip install -r requirements.txt
  ```

### Remote Control

Currently, SamBot only supports remote operation through keyboard. 

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [ ] D435i Integration
- [ ] RPLidar A1 Integration
- [ ] Gazebo Setup

See the [open issues](https://github.com/brandondunbar/SamBot/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Any contributions are appreciated!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the GPLv3 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Brandon Dunbar - brandon.dunbar97@gmail.com

Project Link: [https://github.com/brandondunbar/SamBot/issues](https://github.com/brandondunbar/SamBot/issues)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
