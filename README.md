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
The **Dynamic Systems Framework Engine (DSFE)** is a final year Computer Science project developed to support the design, simulation, and analysis of numerical integration methods under varying dynamic conditions in single or many-body systems.
<br />

Although physically grounded, DSFE **prioritises numerical-transparency first**, and visualisation second. The framework is designed for researching mathematic models through controlled numerical simulations, producing reproducible amd quantitative output data while remaining consistent with physically-valid system and enivronment parameters.
<br />

DSFE integrates real-time visualisation using **OpenGL**, **GLSL**, **GLM**, and **ImGui** to introduce interpretable representations of system behaviour. The visual layer is an addition with the intention of complementing, not replacing, the underlying numerical analysis.
<br />

The framework also includes the **Dynamic Systems Language (DSL)**, a domain-specific scripting langauge that enables exact experiment reproduction, parameter control, and determinisitic test execution - one of the biggest reasons for adding DSL.
<br />

As an independant an extensible final year project, DSFE allows users to:
 * Define custom dynamic systems
 * Import external models
 * Design their own DSL scripts
 * Build their own experimental regimes
 * Run in either batch or GUI mode
 * Run parallel instances without file restrictions

This open architecture intentionally avoids hidden "Black-Box" abstractions, and instead `favours transparency, reproducibility, and research flexibility`.
<br />

> **Disclaimer:**<br />
> The DSFE software has been developed, released, and maintained soley by me([@SaltyJoss](https://github.com/SaltyJoss)). <br />
> If you identify any significant bugs, logical inconsistencies, implementation errors, or any citation issues, please open an Issue or contact me directly via GitHub.<br />
> Constructive feedback and technical corrections are welcomed, as they contribute to improving both the software and my understanding of the subject matter.

<!-- Project Motivation -->
## Project Motivation:
While DSFE originated as a final-year project, it reflects a sustained interest in computational mathematics, computational physics, and their application to space-oriented dynamical systems.<br />
I hope that this repository will continue to evolve as the underlying numerical methods, modelling strategies, and experimental frameworks are refined and extended.
This project forms part of a broader and continuing exploration into how mathematical models govern the behaviour of physically motivated dynamical simulation environments.<br />

<!-- INSTALLATION -->
## Installation

### Pre-Requisites:
Before donwloading the latest release, you will need to have installed the following:
* <a href="https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170">Latest Visual C++ Redistributable</a> <br />

### DSFE Installation:
Download the latest release of DSFE from `Release` in the `DSFE GitHub Repository`:
* <a href="https://github.com/SaltyJoss/RoboticArm_MathModelling/releases">Latest Release of DSFE</a> <br />
<br />

<!-- RUNNING DSFE -->
## Running DSFE

<!-- v0.8.0r-alpha and below -->
### Versions v0.8.0r-alpha and below

#### As an executable
 * Download and extract the files from the [latest release of DSFE](https://github.com/SaltyJoss/RoboticArm_MathModelling/releases) and run the executable (`Engine.exe` on Windows) as you would any other binary!

#### Batch mode (CLI)
 * Open a terminal in the extracted release folder and run the engine in ```--batch``` mode to execute a DSL script that automates simulation sweeps.

```bash
# Show help / usage
PS C:\Users\SaltyJoss\dsfe-v0.7.1r-alpha-windows-x64> .\Engine.exe -h

# Example: one of the commands used to run my tests:
PS C:\Users\SaltyJoss\dsfe-v0.7.1r-alpha-windows-x64> .\Engine.exe --batch -t assets/DSLScripts/vispa_report_test_1.dsl --name vispa_rA_t1 --basedt 1/960 --baseint rk4 --dt 1/30,1/60,1/120,1/240,1/480 --int euler,midpoint,heun,ralston,rk4,rk45
```
<!-- New releases -->
### Current Release (vx.x.x-beta)

#### To be Added...

<!-- PROJECT INFO -->
## Project Info

### Languages used
[![C][C]][C-url]
[![C++][C++]][C++-url]
[![OpenGL][OpenGL]][OpenGL-url]
[![GLSL][GLSL]][GLSL-url]
[![MATLAB][MATLAB]][MATLAB-url]
[![JSON][JSON]][JSON-url]

### Robotic Models used in DSFE:
  * [Google Deepmind's][mujoco-repo] open source models ([MuJoCo][mujoco-url]) of `Z1`, `UR5e`, `iiwa14`, `Panda`, and `H1` robotic systems
  * [StanforASL's][stanfordASL-repo] repo for `Panda` robotic arm
  * [Unitree Robotics's][unitree-ros-repo] repo for the `Z1` and `UR5e` robotic arms
  * [Airbus's][vispa-repo] repo for the their Versatile In-Space and Planetary Arm (`VISPA`) URDF and model files

</div>

<br />

<!-- TODO -->
## Roadmap Ideas:

> **IMPORTANT**: <br />
> As of Release `v0.8.0r-alpha`, this project has had a large refactor. I have now finished my degree, and plan to use DSFE for my masters project, therefore this framework will recieve meaningful updates. <br />
> These updates beging with the separation of concerns, and implementing a CMake-only pipeline. This is the first time I have actively used CMake is such a capacity, meaning if you find ANY bugs or issues please let me know. <br />

### Todo List:
 * [ ] Implement `collision meshes` with existing dynamics pipeline. **(CORE)**
 * [ ] Improve CLI and GUI layouts, making them more `Research-Oriented`. **(CORE)**
 * [ ] Add workspace layouts in GUI mode. **(VISUAL)**
 * [ ] Support multiple concurrent sessions for GUI mode. **(VISUAL)**
 * [ ] Move simulation data output to a `dedicated Data-specific thread`. **(CORE)**
 * [ ] Similar to the above, but multithread DSFE, not just batch parallelisation (running into bottlenecks on a single thread already). **(CORE)**
 * [ ] Support `multiple articulated systems` within a single simulation instance. **(CORE)**
 * [ ] Extend DSFE to support `other classes of dynamical systems` outside robotic manipulators. **(CORE)**
 * [ ] Further `extend physcial modelling` for different robot models (humanoid, legged). **(CORE)**
 * [ ] Explore `DX11` and `Vulkan` alternatives, not necessarily a good idea but could improve usability on specific systems. **(VISUAL)**
 * [ ] Get DSFE to work on `Linux`. **(CORE)**
 * [ ] Explore `non-x86(x64) instruction set` support (`ARM64`, `RISC-V`). **(CORE)**
 * [ ] Integrate the `standardised URDF XML` alongside or in place of the current DSFE json format. **(CORE)**
 * [ ] Implement solution to current friction model (Seems to be introducing stiffness into RK4/RK45?) - `LPV or Karnopp approach maybe?`. **(MATH)**
 * [ ] Implement more advanced structure-preserving integration methods (`Radau IIA methods`, `High-Order SSPRK methods`, even `higher-stage(and therefore order) GLRK methods`). **(MATH)**
 * [ ] Need to look at using `CRBA` instead of just looping through a mass matrix loop. **(MATH)**
 * [ ] Rework DSL to be fully independent of the framework, rather used by the DSFE framework in a specific way via a internal libraries to further integrate specific features. **(CORE)**

### Completed Tasks:
 * [x] <s>Replace the current diagonal model with the standarised `full-matrix rigid-body model`</s> **(CORE)**
 * [x] <s>Implement unit tests for each method relevant to numerical analysis.</s> **(CORE)**
 * [x] <s>Implemented `basic implicit/structure-preserving integrators` (`Implicit Euler`, `Implicit Midpoint`, `GLRK-variants`)</s> **(MATH)**
 * [x] <s>Sepearate core physics/mathematics logic from the GUI and visualisation layers</s> **(CORE)**
 * [x] <s>Explore migration to a `CMake-only` build system</s> **(CORE)**
 * [x] <s>Migrate to a `CMake-only` build system</s> **(CORE)**
 * [x] <s>Further separate the core simulation stepping from the physics/mathematical backend and form the GUI/visualisation layers</s> **(CORE)**

<br />

<!-- LICENSE -->
<div align="center">

## Citations
If you use this software in academic work or any published research, please cite it. <br />
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
* [SSAO logic][SSAO_url] was taken and derived from [SemiWaker][SemiWaker_url]'s SSAO repo - please remember this for any logic SSAO-related.

<p align="center">(<a href="#readme-top">back to top</a>)</p>

## Other Disclaimers:

> **AI Usage Disclaimer:**<br />
> AI tool were used in limited and defined ways throughout this project:<br />
> * Assisstance in finalising PBR and IBL GLSL shader implementations
> * Debugging support after my own attempts using logical analysis, documentation, academic references, and technical forums
> * Early-stage resource discovery and outline (e.g. helping identify relevant literature and refine search queries)
>
> Outside of the GLSL shader implementations, any AI-assisted output was limited to implementation guidance and debugging suggestions. All such suggestions were ritically evaluated and verified prior to usage, with any fixes being developed and implemented by myself.

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
[vispa-repo]: https://github.com/AirbusDefenceAndSpace/vispa
[saltyjoss]: https://github.com/SaltyJoss
[citation-url]: https://github.com/SaltyJoss/RoboticArm_MathModelling/blob/Main/CITATION.cff
[SSAO_url]: https://github.com/semiwaker/SSAO_term_project
[SemiWaker_url]: https://github.com/semiwaker
