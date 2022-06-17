# Git Sub-Trees

This repository consists of multiple git subtrees to combine the multi-repo structure used by Autoware into a single structure for CARMA. These subtrees are created via the `git subtree` command originally cloning the 1.12 version of each of the Autoware.ai repositories on [GitLab](https://gitlab.com/autowarefoundation/autoware.ai) as well as additional repositories identified as dependencies in the `autoware.ai.repos` file in the `autoware` repository. The following procedures in this file can be followed to update each of these repositories from the main Autoware.ai remotes. Additional information on subtrees can be found [here](https://blog.developer.atlassian.com/the-power-of-git-subtree/).

# NOTICE: When working with subtrees please ensure that individual commits only change files in *ONE-AND-ONLY-ONE* subtree.
If your branch must change multiple subtrees please make those changes in separate commits. Do not squash commits that change multiple subtrees, even when merging via Github.

## Add remotes

### Main Autoware Repositories

```shell
git remote add core_planning_remote git@gitlab.com:autowarefoundation/autoware.ai/core_planning.git
git remote add simulation_remote git@gitlab.com:autowarefoundation/autoware.ai/simulation.git
git remote add utilities_remote git@gitlab.com:autowarefoundation/autoware.ai/utilities.git
git remote add visualization_remote git@gitlab.com:autowarefoundation/autoware.ai/visualization.git
git remote add core_perception_remote git@gitlab.com:autowarefoundation/autoware.ai/core_perception.git
git remote add drivers_remote git@gitlab.com:autowarefoundation/autoware.ai/drivers.git
git remote add common_remote git@gitlab.com:autowarefoundation/autoware.ai/common.git
git remote add messages_remote git@gitlab.com:autowarefoundation/autoware.ai/messages.git
git remote add documentation_remote git@gitlab.com:autowarefoundation/autoware.ai/documentation.git
git remote add autoware_remote git@gitlab.com:autowarefoundation/autoware.ai/autoware.git
```

### Autoware Dependencies

```shell
git remote add lanelet2_remote git@github.com:fzi-forschungszentrum-informatik/Lanelet2.git
git remote add mrt_cmake_modules_remote git@github.com:KIT-MRT/mrt_cmake_modules.git
git remote add qpoases_remote git@gitlab.com:autowarefoundation/autoware.ai/qpoases_vendor.git
git remote add car_demo_remote git@github.com:CPFL/car_demo.git
git remote add ds4_remote git@github.com:tier4/ds4.git
git remote add osrf_citysim_remote git@github.com:CPFL/osrf_citysim.git
git remote add jsk_recognition_remote git@github.com:jsk-ros-pkg/jsk_recognition.git
git remote add jsk_common_msgs_remote git@github.com:jsk-ros-pkg/jsk_common_msgs.git
```

## Pull from remotes

### Main Autoware Repositories

```shell
git subtree pull --prefix=core_planning core_planning_remote master --squash
git subtree pull --prefix=simulation simulation_remote master --squash
git subtree pull --prefix=utilities utilities_remote master --squash
git subtree pull --prefix=visualization visualization_remote master --squash
git subtree pull --prefix=core_perception core_perception_remote master --squash
git subtree pull --prefix=drivers drivers_remote master --squash
git subtree pull --prefix=common common_remote master --squash
git subtree pull --prefix=messages messages_remote master --squash
git subtree pull --prefix=documentation documentation_remote master --squash
git subtree pull --prefix=autoware autoware_remote master --squash
```

### Autoware Dependencies

```shell
git subtree pull --prefix=lanelet2 lanelet2_remote master --squash
git subtree pull --prefix=mrt_cmake_modules mrt_cmake_modules_remote master --squash
git subtree pull --prefix=qpoases qpoases_remote master --squash
git subtree pull --prefix=car_demo car_demo_remote master --squash
git subtree pull --prefix=ds4 ds4_remote master --squash
git subtree pull --prefix=osrf_citysim osrf_citysim_remote master --squash
git subtree pull --prefix=jsk_recognition jsk_recognition_remote 1.2.15 --squash
git subtree pull --prefix=jsk_common_msgs jsk_common_msgs_remote 4.3.2 --squash
```

## Adding Sub-Trees

```shell
git remote add <subtree_name>_remote git@github.com:<subtree repo>.git
git subtree add --prefix=<subtree_name> <subtree_name>_remote master --squash
```
