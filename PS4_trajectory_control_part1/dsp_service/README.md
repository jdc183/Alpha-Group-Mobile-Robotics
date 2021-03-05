# Mobot Path Execution, Part 1--Alpha Group, Mobile Robotics
This is the upload for PS4: Mobot Path Execution Part 1 for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

This project consists two packages that work together in order to have the mobot behave as though it is wall following in order to exit the starting pen. 

## Additional Service Functionality
This package includes an additional service, named dsp_service. This package includes the server needed for the desired-state publisher, and must be included in the catkin_ws alongside the mobot_path_execution package in order to ensure proper operation. Included in this package is one service.

### DSPService.srv
This service allows the desired-state publisher service to operate correctly as a service. It takes a PoseStamped as its request for a goal state, and sends a boolean response to allow the navigation coordinator to know when to send the next goal state.

## Operation
There are no additional instructions to run this service. Simply run the primary package, and this service will do its job.

