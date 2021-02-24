// Nicole Graf, Joseph Cressman, Andrew Capelli

// This node will be upgraded to perform “lane-drift” 
// correction, heading control and path progress control.  
// For this assignment, implement this equivalent to the 
// open-loop controller, which merely copies a desired state 
// twist to cmd_vel.  The controller should be modified to
// prepare for different control modes: spin-in-place, 
// straight-line-motion, and halt (to be implemented later).