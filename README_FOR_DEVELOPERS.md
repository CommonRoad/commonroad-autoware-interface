# CR2AW: Additional Information for Developers

## Best-Practices for modifications and extensions

### Rebuilding the node

Any time you make modifications to the code of CR2AW, you need to rebuild the node using colcon:

```shell
   cd <AUTOWARE_ROOT_DIRECTORY>
   colcon build --symlink-install --packages-select cr2autoware
```
, where `<AUTOWARE_ROOT_DIRECTORY>` should be replaced accordingly to your setup.

In addition, it is advised to always source the workspace after rebuilding via:

```shell
   source install/setup.bash
```


### Adding new modules/dependencies

In case you want to extend the codebase by adding new modules or dependencies, we recommend the following procedure:


**PyPi packages**

New Pip dependencies can be added to the `requirements.txt` file. Afterwards re-run the setup bash-script `cr2autoware_install.sh`
for CR2AW

```shell
   cd src/universe/autoware.universe/planning/tum_commonroad_planning/commonroad-autoware-interface/
   ./cr2autoware_install.sh
   ```


**Git Repos**

New dependencies/modules which should be pulled as source code from a Github/Gitlab repository can be added to the 
`tum.commonroad.planning.repos` file. They can then be pulled via VCS by re-running the command:

```shell
   vcs import src < tum.commonroad.planning.repos
```

Additionally, in case the module requires a specific setup procedure (e.g., building/installing), the setup bash script 
`cr2autoware_install.sh` can be extended accordingly.



## CR2AW with Docker-Compose Setup

**NOTE: This is only relevant for TUM-internal developers who use Autoware.Universe within a 
docker-compose (microservice) architecture.**

Please refer to the [tum_launch](https://gitlab.lrz.de/av2.0/tum_launch) where the setup and launch of Autoware.Universe
within the microservice architecture is located.

To setup CR2AW within the planning microservice, one can proceed as follows:
* Start the planning microservice docker image 
  ```shell
  docker run -it <PLANNING_IMAGE_TAG>
  ```
* Perform the setup steps described in the [README](README.md)
* Commit the docker container:
  ```shell
  docker commit <CONTAINER_ID> <PLANNING_IMAGE_TAG>
  ```
* Re-launch Autoware.Universe via Docker compose. E.g., to launch the PlanningSim
  ```shell
  docker compose -f planning_sim.yml --env-file .env up
  ```

