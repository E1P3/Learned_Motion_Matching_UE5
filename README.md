Learned Motion Matching implementation for Unreal Engine 5. Part of my [Master Thesis](https://github.com/E1P3/Learned_Motion_Matching_UE5/blob/main/thesis.pdf)
.
The project features a fully implemented LMM character controller for ground locomotion.

Tools for training the neural networks that are used by this controller are available [here](https://github.com/E1P3/Learned_Motion_Matching_Training)

To implement the LMM into your project, copy the files located in [Private](https://github.com/E1P3/Learned_Motion_Matching_UE5/tree/main/Source/Testing/Private) and [Public](https://github.com/E1P3/Learned_Motion_Matching_UE5/tree/main/Source/Testing/Private) and you will be able to add "Learned Motion Matching" node into your character's animation graph. The node itself requires the necessarry models (which can be trained using [this repository](https://github.com/E1P3/Learned_Motion_Matching_Training)) as well as the Character Trajectory component, that can be added in the same manner as shown [here](https://www.youtube.com/watch?v=L2Q8C99uYuE). 

Side note: Run this project from the IDE in debug mode rather then in development mode. For some reason the file parser does not work natively.

Enjoy :)
