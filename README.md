# Autoware.ai
This is a fork of Autoware containing modifications to support usage with the [CARMAPlatform](https://github.com/usdot-fhwa-stol/CARMAPlatform). This repository contains changes to the Autoware source code and configuration that may not be supported by the Autoware Foundation and may not be consistent with the original design intent of Autoware. All modifications in this repository are licensed under the same Apache License 2.0 as Autoware and all modifications of the source code made will be marked as such in accordance with the terms of the Apache License 2.0. For a list of modifications and their descriptions please see [NOTICE.md](NOTICE.md).

The next several sections of this readme provide standard information on how this repo relates to the rest of the CARMA project. Below that is the full content of the readme from the master Autoware repo.

### For developers working in this repository:
For any modified file please follow these steps to ensure proper documentation of this modification in compliance with the terms of the Apache License 2.0:

1. Add a comment at the top of any modified file with a high-level description of the modification and date the modification was made.
2. Add a high-level description and date of the overall modification to the [NOTICE.md](NOTICE.md) file.

### Local CARMA mods to this readme file:
- Added notice section describing fork status and providing instructions for developers
  - 5/10/2019
  - Kyle Rush
- Added several intro sections to provide consistency in appearance among all CARMA repos
  - 10/11/2019
  - John Stark

# CARMAPlatform
The primary CARMAPlatform repository can be found [here](https://github.com/usdot-fhwa-stol/CARMAPlatform) and is part of the [USDOT FHWA STOL](https://github.com/usdot-fhwa-stol/)
github organization. Documentation on how the CARMAPlatform functions, how it will evolve over time, and how you can contribute can be found at the above links as well

## Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/Contributing.md) 

## Code of Conduct 
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/ATTRIBUTION.txt) 

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/docs/License.md) 

## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/CARMAPlatform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)



## Autoware
[![Autoware](https://www.autoware.ai/static/img/autoware_web_img.png)](https://www.autoware.ai)

[Autoware](https://www.autoware.ai) is the world's first "all-in-one" open-source software for self-driving vehicles. The capabilities of Autoware are primarily well-suited for urban cities, but highways, freeways, mesomountaineous regions, and geofenced areas can be also covered. The code base of Autoware is protected by the Apache 2 License. Please use it at your own discretion. For safe use, we provide a ROSBAG-based simulation environment for those who do not own real autonomous vehicles. If you plan to use Autoware with real autonomous vehicles, **please formulate safety measures and assessment of risk before field testing.**

You may refer to [Autoware Wiki](https://github.com/CPFL/Autoware/wiki) for **Users Guide** and **Developers Guide**.

## What Is Autoware

[![Autoware Overview](docs/images/autoware_overview.png)](https://github.com/CPFL/Autoware/wiki/Overview)

Autoware provides a rich set of self-driving modules composed of sensing, computing, and actuation capabilities. An overview of those capabilities is described [here](https://github.com/CPFL/Autoware/wiki/Overview). Keywords include *Localization, Mapping, Object Detection & Tracking, Traffic Light Recognition, Mission & Motion Planning, Trajectory Generation, Lane Detection & Selection, Vehicle Control, Sensor Fusion, Cameras, LiDARs, RADARs, Deep Learning, Rule-based System, Connected Navigation, Logging, Virtual Reality, and so on*.

Free manuals can be also found at [Autoware-Manuals](https://github.com/CPFL/Autoware-Manuals). You are encouraged to contribute to the maintenance of these manuals. Thank you for your cooperation!

## Getting Started

[![Autoware Demo](docs/images/autoware_demo.png)](https://github.com/CPFL/Autoware/wiki/Demo)

### Recommended System Specifications

- Number of CPU cores: 8
- RAM size: 32GB
- Storage size: 64GB+

### Users Guide

1. [Installation](https://github.com/CPFL/Autoware/wiki/Installation)
    1. [Docker](https://github.com/CPFL/Autoware/wiki/Docker)
    1. [Source](https://github.com/CPFL/Autoware/wiki/Source-Build)
1. [Demo](https://github.com/CPFL/Autoware/wiki/Demo)
1. [Field Test](https://github.com/CPFL/Autoware/wiki/Field-Test)
1. [Simulation Test](https://github.com/CPFL/Autoware/wiki/Simulation-Test)
1. [Videos](https://github.com/CPFL/Autoware/wiki/videos)

### Developers Guide

1. [Contribution Rules](https://github.com/CPFL/Autoware/wiki/Contribution-Rules) (**Must Read**)
1. [Overview](https://github.com/CPFL/Autoware/wiki/Overview)
1. [Specification](https://github.com/CPFL/Autoware/wiki/Specification)


## Research Papers for Citation

1. S. Kato, S. Tokunaga, Y. Maruyama, S. Maeda, M. Hirabayashi, Y. Kitsukawa, A. Monrroy, T. Ando, Y. Fujii, and T. Azumi,``Autoware on Board: Enabling Autonomous Vehicles with Embedded Systems,'' In Proceedings of the 9th ACM/IEEE International Conference on Cyber-Physical Systems (ICCPS2018),  pp. 287-296, 2018. [Link](https://dl.acm.org/citation.cfm?id=3207930)

2. S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, and T. Hamada. ``An Open Approach to Autonomous Vehicles,'' IEEE Micro, Vol. 35, No. 6, pp. 60-69, 2015. [Link](https://ieeexplore.ieee.org/document/7368032/)

## Cloud Services

### Autoware Online

You may test Autoware at [Autoware Online](http://autoware.online/). No need to install the Autoware repository to your local environment.

### Automan

You may annotate and train your ROSBAG data using your web browser through [Automan](https://www.automan.ai). The trained models can be used for deep neural network algorithms in Autoware, such as SSD and Yolo.

### ROSBAG STORE

You may download a number of test and simulation data sets from Tier IV's [ROSBAG STORE](https://rosbag.tier4.jp). Note that free accounts would not allow you to access image data due to privacy matters. 

### Map Tools

You may create 3D map data through Tier IV's [Map Tools](https://maptools.tier4.jp/). The 3D map data used in Autoware are composed of point cloud structure data and vector feature data.

## License

Autoware is provided under the [Apache 2 License](https://github.com/CPFL/Autoware/blob/master/LICENSE).

## Contact

[Autoware Developers Slack Team](https://autoware.herokuapp.com/)

[Autoware Discourse](https://discourse.ros.org/c/autoware)

Please see the [Support Guidelines](https://github.com/CPFL/Autoware/wiki/Support-guidelines) for more details about getting help.

***
<div align="center"><img src="docs/images/autoware_logo_1.png" width="400"/></div>
