# Welcome to uml-robotics' fork of spot_ros!
## Current Changelist
1. ListTaggedObjects: service for listing the ids of fiducials spot currently sees
2. GetObjectPose: service for getting the pose of a fiducial given its ID
3. Services for undocking and docking spot
4. Service for localizing spot in a downloaded GraphNav map
5. Publisher for localization state
6. Converted gripper angle move service to an ActionServer
7. Various changes to services, actions, and urdf from https://github.com/estherRay/spot_ros to get support for Spot's arm
