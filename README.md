<a id="readme-top"></a>

<div align="center">

<img width="950" height="105" alt="image" src="https://github.com/user-attachments/assets/9e48b947-a16c-4cd1-92ff-5a357f21fd30" />

![Build Status][build-shield]
[![License][license-shield]][license-url] 
![Commit Activity][commit-activity-shield]
![Last Commit][last-commit-shield]
![Code Size][size-shield]
![Top Language][top-language-shield]
![Language Count][language-count-shield]
![Repo Stars][repo-stars-shield]
![Watchers][watchers-shield]

<img width="800" height="53" alt="image" src="https://github.com/user-attachments/assets/9e90197b-d53d-4bb5-bb01-0d252da06950" />

</div>

<div align="left">

<!-- ABOUT THE PROJECT -->
## About DSFE
<br />
<br />

<!-- INSTALLATION -->
## Installation

### Pre-Requisites:
Before donwloading the latest release, you will need to have installed the following:
* <a href="https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170">Latest Visual C++ Redistributable</a> <br />

### DSFE Installation:
Download the latest release of DSFE from `Release` in the `DSFE GitHub Repository`:
* <a href="https://github.com/SaltyJoss/RoboticArm_MathModelling/releases">Latest Release of DSFE</a> <br />
<br />

> **Disclaimer:**<br />
> The DSFE software has been developed, maintained, and released by [@SaltyJoss](https://github.com/SaltyJoss). As it is still under development, please submit and issue or comment if you find any major bugs/issues/errors within the software.

<!-- PROJECT INFO -->
## Project Info

### Languages used
[![C][C]][C-url]
[![C++][C++]][C++-url]
[![OpenGL][OpenGL]][OpenGL-url]
[![GLSL][GLSL]][GLSL-url]
[![MATLAB][MATLAB]][MATLAB-url]
[![JSON][JSON]][JSON-url]

### Robotic Models Used:
  * [Google Deepmind's][mujoco-repo] open source models ([MuJoCo][mujoco-url]) of `Z1`, `UR5e`, `iiwa14`, `Panda`
  * [StanforASL's][stanfordASL-repo] repo for `Panda` robotic arm
  * [Unitree Robotics's][unitree-ros-repo] for the `Z1` and `UR5e` robotic arms

</div>

<br />

<!-- LICENSE -->
<div align="center">

## Citations
If you use this software in academic work, please cite it. <br />
See [`CITATION.cff`][citation-url].

## License
Licensed under the [GPL-3.0 License][license-url].

> **TL;DR:** <br />
> This project is licensed under **GPL-3.0**. You may **use, modify, and distribute** it (including commercially). <br />
> If you **distribute** this project or a modified version, you must **provide the corresponding source code** under **GPL-3.0**, and **keep copyright, license, and attribution notices** intact. <br />
> Modified versions should be **clearly marked as modified**. See the [LICENSE][license-url] file for details. <br />

<br />

> **Attribution:** Please retain the [original author][saltyjoss] credit. A link back to this repository is greatly appreciated. <br />

</div>

## Third-Party Tools Used:
The software itself is made of 3 seperate solutions that are under the GPL-3.0 License. However, it has been built and tested using various thirdparty tools and libraries.
* [Eigen][Eigen-GitLab] for high-performance linear algebra operations that compute robot kinematics, dynamics, and numerical integration
* [GLM][GLM-url] for OpenGL-compatible vector and matrix types for graphics transforms, camera math, and rendering-side calculations
* [GLFW][GLFW-url] for creating windows, manage OpenGL contexts, and handle user input across platforms
* [GLAD][GLAD-url] for loading OpenGL function pointers at runtime, and enabling access to more modern OpenGL features
* [ImGui][ImGui-url] for immediate-mode GUI for runtime controls, debugging panels, simulation visualisation tools, and the DSL script editor
* [ImPlot][ImPlot-url] for more complex realtime plots used during simulation runs in DSFE
* [Assimp][Assimp-url] for importing 3D meshes and scene data from many file formats for robot models and misc objects
* [nlohmann Json][Nlohmann-Json-url] for integration of .json files within c++, allowing robots to have defined properties that can be easily retrieved upon loading
* [HDF5][HDF5-url] for storing structured time-series data from robotic arm simulation runs for post-analysis and comparisons of integration methods performance and stability
* [stb image][stb-image-url] for lightweight loading of png and jpgs files previously used in cubemaps

<p align="center">(<a href="#readme-top">back to top</a>)</p>

<!-- PROJECT BADGES -->
[build-shield]: https://img.shields.io/github/actions/workflow/status/SaltyJoss/RoboticArm_MathModelling/build.yml?style=for-the-badge

[commit-activity-shield]: https://img.shields.io/github/commit-activity/t/SaltyJoss/RoboticArm_MathModelling.svg?style=for-the-badge
[last-commit-shield]: https://img.shields.io/github/last-commit/SaltyJoss/RoboticArm_MathModelling.svg?style=for-the-badge
[top-language-shield]: https://img.shields.io/github/languages/top/SaltyJoss/RoboticArm_MathModelling.svg?style=for-the-badge
[language-count-shield]: https://img.shields.io/github/languages/count/SaltyJoss/RoboticArm_MathModelling.svg?style=for-the-badge
[size-shield]: https://img.shields.io/github/languages/code-size/saltyjoss/RoboticArm_MathModelling.svg?style=for-the-badge
[license-shield]: https://img.shields.io/badge/GitHub-GPL--3.0-green.svg?style=for-the-badge
[license-url]: https://github.com/SaltyJoss/RoboticArm_MathModelling/blob/Main/LICENSE
[repo-stars-shield]: https://img.shields.io/github/stars/saltyjoss/RoboticArm_MathModelling.svg?style=for-the-badge
[watchers-shield]: https://img.shields.io/github/watchers/saltyjoss/RoboticArm_MathModelling.svg?style=for-the-badge

<!-- LANGUAGE BADGES -->
[C++]: https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=C%2B%2B&logoColor=white
[C++-url]: https://isocpp.org/
[C]: https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white
[C-url]: https://en.cppreference.com/w/c/language
[OpenGL]: https://img.shields.io/badge/OpenGL-50C878?style=for-the-badge&logo=opengl&logoColor=white
[OpenGL-url]: https://www.opengl.org/
[GLSL]: https://img.shields.io/badge/GLSL-50C878?style=for-the-badge&logo=opengl&logoColor=white
[GLSL-url]: https://www.khronos.org/opengl/wiki/Core_Language_(GLSL)
[MATLAB]: https://img.shields.io/badge/MATLAB-FF2323?style=for-the-badge&logo=mathworks&logoColor=white
[MATLAB-url]: https://www.mathworks.com/products/matlab.html
[JSON]: https://img.shields.io/badge/JSON-5ACB00?style=for-the-badge&logo=json&logoColor=white
[JSON-url]: https://www.json.org/json-en.html

<!-- LIBRARY BADGES -->
[GLFW-url]: https://www.glfw.org/
[GLAD-url]: https://github.com/Dav1dde/glad
[GLM-url]: https://github.com/g-truc/glm
[ImGui-url]: https://github.com/ocornut/imgui
[ImPlot-url]: https://github.com/epezent/implot
[Assimp-url]: https://github.com/assimp/assimp
[Eigen-url]: https://libeigen.gitlab.io/
[Eigen-GitLab]: https://gitlab.com/libeigen/eigen
[Nlohmann-Json-url]: https://github.com/nlohmann/json
[HDF5-url]: https://github.com/HDFGroup/hdf5
[stb-image-url]: https://github.com/nothings/stb/blob/master/stb_image.h

<!-- EXTRA LINKS -->
[mujoco-repo]: https://github.com/unitreerobotics/unitree_ros
[mujoco-url]: https://mujoco.org/
[stanfordASL-repo]: https://github.com/StanfordASL/PandaRobot.jl
[unitree-ros-repo]: https://github.com/unitreerobotics/unitree_ros
[saltyjoss]: https://github.com/SaltyJoss
[citation-url]: https://github.com/SaltyJoss/RoboticArm_MathModelling/blob/Main/CITATION.cff
