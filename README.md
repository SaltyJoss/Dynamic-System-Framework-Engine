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
The **Dynamic Systems Framework Engine (DSFE)** is a computational framework that supports the design, simulation, and analysis of numerical integration methods under varying dynamic conditions in single- and many-body systems.
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
> <br />
> Constructive feedback and technical corrections are welcomed, as they contribute to improving both the software and my understanding of the subject matter.

<!-- Project Motivation -->
## Project Motivation:
While DSFE originated as a final-year project, it reflects a sustained interest in computational mathematics, computational physics, and their application to space-oriented dynamical systems.<br />
I hope that this repository will continue to evolve as the underlying numerical methods, modelling strategies, and experimental frameworks are refined and extended.
This project forms part of a broader and continuing exploration into how mathematical models govern the behaviour of physically motivated dynamical simulation environments.<br />

<!-- INSTALLATION -->
## Installation

### For DSFE GUI users only:
  #### **<u>Pre-Requisites:</u>**
  Before donwloading the latest release, you will need to have installed the following: <br />
  ##### **Windows(x86/64):**
  * <a href="https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170">Latest Visual C++ Redistributable</a> <br />
  
  ##### **Linux(x86/64):**

  * Upto-date C++ compiler (GCC or Clang) <br />
  * **GCC:**
    * Debian-based Distributions:

      ```
      sudo apt update
      sudo apt install build-essential gdb
      ```

    * Red Hat-based systems:

      ```
      sudo dnf check-update
      sudo dnf install gcc-c++ gdb
      ```

    * Arch-based Distributions:

      ```
      sudo pacman -Syu
      sudo pacman -S base-devel gdb
      ```

  * **CLang:**
    * Debian-based Distributions: ```sudo apt install clang```

    * Red Hat-based systems: ```sudo dnf install clang```

    *  Arch-based Distributions: ```sudo pacman -S clang```

  #### **<u>DSFE Installation:</u>**
  Download the latest release of DSFE from `Release` in the `DSFE GitHub Repository`:
  * <a href="https://github.com/SaltyJoss/RoboticArm_MathModelling/releases">Latest Release of DSFE</a> <br />
<br />

### For user planning to add/extend/edit the underlying code:

> To be added...

<!-- RUNNING DSFE -->
## Running DSFE

<!-- v0.8.0r-alpha and below -->
### Versions v0.8.0r-alpha and below

#### As an executable
 * Download and extract the files from the [latest release of DSFE](https://github.com/SaltyJoss/Dynamic-System-Framework-Engine/releases) and run the executable (`DSFE_App.exe` on Windows) as you would any other binary!

#### Batch mode (CLI)
 * Open a terminal in the extracted release folder and run the engine in ```--batch``` mode to execute a DSL script that automates simulation sweeps.

```bash
# Show help / usage
PS C:\Users\SaltyJoss\dsfe-v0.7.1r-alpha-windows-x64> .\Engine.exe -h

# Example: one of the commands used to run my tests:
PS C:\Users\SaltyJoss\dsfe-v0.7.1r-alpha-windows-x64> .\Engine.exe --batch -t assets/DSLScripts/vispa_report_test_1.dsl --name vispa_rA_t1 --basedt 1/960 --baseint rk4 --dt 1/30,1/60,1/120,1/240,1/480 --int euler,midpoint,heun,ralston,rk4,rk45
```
> More Info to be added soon...

<!-- New releases -->
### Current Release (vx.x.x-beta)

#### To be Added...

<!-- PROJECT INFO -->
## Project Info

### Languages and Frameworks used
[![C][C]][C-url]
[![C++][C++]][C++-url]
[![JSON][JSON]][JSON-url]
[![VulkanSDK][VulkanSDK]][Vulkan-url]
[![GLSL][GLSL]][GLSL-url]

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
> After the release of `v0.8.0r-alpha` this project will be refactored heavily to improve the general pipeline, functionality, and usability for research. This also means DSFE(Core) and DSFE(App) will move away from `single-threaded` to `multi-threaded` practices. <br />
> On May 8th 2026 my degree content/examinations are finished, meaning DSFE will be prioritised again. It is important to understand that the roadmap of this project outlines the larger plans, with many smaller ones not being added. <br />
> <br />
> In September 2026 I start my masters in Computer Science, for which I plan to use DSFE for the final research project in 2027. This means DSFE will become more generalised in application, with core library (DSFE_Core) aiming to be used for multi-disciplinary research applying numerical models. <br />
> <br />
> These changes have begun with the move to CMake-only building and compiling, alongside separation of concerns for DSFE's core library, and the app-specific executable DSFE_Engine. Originally, the plan was to only involve white-box applications, but given the core library is openly available for use external to the application, it makes sense to include a pre-compiled and functional application for researchers and academics who do not wish to code in C++. However, the DSL (Dynamic Systems Language) component of DSFE will be necessary for writing tests and experiments that are runnable. I will work on getting a gitpages documentation site out for it once the more immediate changes have been made. DSL also has plans to evolve from its current state. <br />
> <br />
> I am the single developer of DSFE, being self-taught in higher-mathematics and computational physics, so please understand that there will be mistakes and errors in the code. If you find any areas of improvement, please let me know. I want this to become a functional and useful research tool for numerical modelling. <br />

### Todo List:
 * [ ] Implement `collision meshes` with existing dynamics pipeline. **(CORE)**
 * [ ] Improve CLI and GUI layouts, making them more `Research-Oriented`. **(CORE)**
 * [ ] Move simulation data output to a `dedicated Data-specific thread`. **(CORE)**
 * [ ] Explore `non-x86(x64) instruction set` support (`ARM64`, `RISC-V`). **(CORE)**
 * [ ] Integrate the `standardised URDF XML` alongside or in place of the current DSFE json format. **(CORE)**

### Tasks in progress:
> This is what I am actively implementing, not just planning to implement
 * [ ] Explore CUDA benefits in the Core and PxM libraries (Keyword here is explore, this is a discovery before execution) **(Core)**
 * [ ] Extend DSFE to support `other classes of dynamical systems` outside robotic limbs (looking at end-effector grabbers, single-body systems like particles or celestrial object, and more). **(CORE)**
 * [ ] Support `multiple articulated systems` within a single simulation instance. **(CORE)**
 * [ ] Further `extend physcial modelling` for different systems (humanoid, legged, single-bodied). **(CORE)**
 * [ ] Rework DSL to be fully independent of the framework, rather used by the DSFE framework in a specific way via a internal libraries to further integrate specific features. **(CORE)**
 * [ ] Implement more advanced structure-preserving integration methods (`Radau IIA methods`, `High-Order SSPRK methods`, even `higher-stage(and therefore order) GLRK methods`). **(MATH)**
 * [ ] Spend some real time on further code cleanup, focuse on refactoring `DSFE_Core`(including the `RobotSystem` disguisting code and actual enforcement of good practices across the library) **(CORE)**
 * [ ] Add workspace layouts in GUI mode. **(VISUAL)**
 * [ ] Support multiple concurrent sessions for GUI mode. **(VISUAL)**
 
### Completed Tasks:
 * [x] <s>Replace the current diagonal model with the standarised `full-matrix rigid-body model` **(CORE)**</s>
 * [x] <s>Implement unit tests for each method relevant to numerical analysis **(CORE)**</s>
 * [x] <s>Implement `basic implicit/structure-preserving integrators` (`Implicit Euler`, `Implicit Midpoint`, `GLRK-variants`) **(MATH)**</s>
 * [x] <s>Sepearate core physics/mathematics logic from the GUI and visualisation layers **(CORE)**</s>
 * [x] <s>Explore migration to a `CMake-only` build system **(CORE)**</s>
 * [x] <s>Migrate to a `CMake-only` build system **(CORE)**</s>
 * [x] <s>Further separate the core simulation stepping from the physics/mathematical backend and form the GUI/visualisation layers **(CORE)**</s>
 * [x] <s>Need to look at using `CRBA` instead of just looping through a mass matrix loop. **(MATH)**</s>
 * [x] <s>Implement more advanced structure-preserving integration methods (`Radau IIA methods`, `High-Order SSPRK methods`, maybe `Automatic Differentitation`). **(MATH)**</s>
 * [x] <s>Implement `RNEA`, `CRBA`, and `ABA` to replace FDM for approximating `M(q)` and `qdd`. **(MATH)**</s>
 * [x] <s>Get DSFE to work on `Linux`. **(CORE)**</s>
 * [x] <s>Explore `DX11` and `Vulkan` alternatives, not necessarily a good idea but could improve usability on specific systems. **(VISUAL)**</s>
 * [x] <s>Replace `OpenGL` with `Vulkan` **(VISUAL)**</s>

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
* [Qt6][Qt6-url] for the widgets and other front/user-facing parts of the graphical interface
* [GLM][GLM-url] for OpenGL-compatible vector and matrix types for graphics transforms, camera math, and all render-specific computation
* [Assimp][Assimp-url] for importing 3D meshes and scene data from many file formats for robot models and misc objects
* [nlohmann Json][Nlohmann-Json-url] for integration of .json files within c++, allowing robots to have defined properties that can be easily retrieved upon loading
* [HDF5][HDF5-url] for storing structured time-series data from robotic arm simulation runs for post-analysis and comparisons of integration methods performance and stability
* [stb image][stb-image-url] for lightweight loading of png and jpgs files previously used in cubemaps
* [SSAO logic][SSAO_url] was taken and derived from [SemiWaker][SemiWaker_url]'s SSAO repo - please remember this for any logic SSAO-related (Pre-Vulkan)

<p align="center">(<a href="#readme-top">back to top</a>)</p>

## Other Disclaimers:

> **AI Usage Disclaimer:**<br />
> AI tool were used in limited and defined ways throughout this project:<br />
> * Assisstance in finalising PBR and IBL GLSL shader implementations in the OpenGL version of DSFE_GUI
> * Assistance with setting up Vulkan for the first time, along with key debugging (made it worse quite alot, but provided a second set of eyes so yeah)
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
[C++]: https://img.shields.io/badge/C++-%2300599C.svg?logo=c%2B%2B&logoColor=white
[C++-url]: https://isocpp.org/
[C]: https://img.shields.io/badge/C-00599C?logo=c&logoColor=white
[C-url]: https://en.cppreference.com/w/c/language
[VulkanSDK]: https://img.shields.io/badge/Vulkan-AE0F28?logo=Vulkan&logoColor=fff
[Vulkan-url]: https://vulkan.lunarg.com/sdk/home
[GLSL]: https://img.shields.io/badge/GLSL-50C878?logo=opengl&logoColor=white
[GLSL-url]: https://www.khronos.org/opengl/wiki/Core_Language_(GLSL)
[JSON]: https://img.shields.io/badge/JSON-000?logo=json&logoColor=fff
[JSON-url]: https://www.json.org/json-en.html

<!-- LIBRARY BADGES -->
[GLM-url]: https://github.com/g-truc/glm
[Assimp-url]: https://github.com/assimp/assimp
[Eigen-url]: https://libeigen.gitlab.io/
[Qt6-url]: https://www.qt.io/development/qt-framework/qt6
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
