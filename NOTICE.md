## Note

This is a fork of Autoware containing modifications to support usage with the [CARMAPlatform](https://github.com/usdot-fhwa-stol/CARMAPlatform). This repository contains changes to the Autoware source code and configuration that may not be supported by the Autoware Foundation and may not be consistent with the original design intent of Autoware. All modifications in this repository are licensed under the same Apache License 2.0 as Autoware and all modifications of the source code made will be marked as such in accordance with the terms of the Apache License 2.0. For a list of modifications and their descriptions please see [NOTICE.md](NOTICE.md).

### For developers working in this repository:

For any modified file please follow these steps to ensure proper documentation of this modification in compliance with the terms of the Apache License 2.0:

1. Add a comment at the top of any modified file with a high-level description of the modification and date the modification was made.
2. Add a high-level description and date of the overall modification to the [NOTICE.md](NOTICE.md) file.

## Modifications:

- Addition of notice about fork status to the README.md and creation of this NOTICE.md file
  - 5/10/2019
  - Kyle Rush
- Added the deadreckoner node from the AutonomouStuff fork of Autoware
  - 5/13/2019
  - Michael McConnell
- Modified points_map_loader package to publish the tf from map to ECEF frame using tf2 library
  - 6/7/2019
  - Shuwei Qiang
- Modified waypoint_loader node to allow reloading a new route csv file using the file path provided by the rostopic
  - 6/24/2019
  - Shuwei Qiang
- Modified points_map_loader package to have option for loading map cells from arealist.txt file directly instead of using redundant file paths as well as improved launch file
  - 6/25/2019
  - Michael McConnell
- Removed exisitng Docker configuration files for replacement with the new CARMA3 docker config files.
  - 8/1/2019
  - Kyle Rush
- Updated as package (ssc_interface) to version found in the AutonomouStuff fork of Autoware
  - 8/6/2019
  - Michael McConnell
- Add carma_autoware_build bash script which builds the portions of autoware needed by carma
  - 8/13/2019
  - Michael McConnell
- Add add new launch file and params yaml file to the ray_ground_filter node
  - 8/20/2019
  - Michael McConnell
- Add a .circleci folder to setup CI for CARMA
  - 9/5/2019
  - Michael McConnell
- Added standard CARMA header to the README.md, while retaining all of the original Autoware content.
  - 10/11/2019
  - John Stark
- Changed entire repo structure to be a collection of subtrees after TierIV's repository refactor and move to Gitlab.
  - 10/30/2019
  - Kyle Rush
- Removed the changes to ray_ground_filter by Michael McConnell on 8/20/2019 as Autoware.ai 1.13 makes them unnecessary 
  - 11/18/2019
  - Michael McConnell
- Undid deletion of the colcon_release script which was removed in Autoware.ai v1.13 as it is still used by the carma_autoware_build script
  - 11/18/2019
  - Michael McConnell

- Updated to revise the way centerline recalulation works to better
  reflect the kinds of maps that CARMA has and to work better with
  the CARMA World Model.
  - 3/11/2020
  - Kyle Rush

