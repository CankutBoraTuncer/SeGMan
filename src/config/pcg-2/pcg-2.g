Include: <../base-walls-min.g>

goal (floor) 	{  Q:[-1.8, -1.8, 0, -1, 0, 0, 0], shape:ssBox, size:[0.35, 0.35, 0.1, 0.005], color:[0.85, 0.25, 0.25, 0.9], logical:{ table } }
goal_visible (floor) 	{  Q:[-1.8, -1.8, 0.1, -1, 0, 0, 0], shape:ssBox, size:[0.35, 0.35, 0.1, 0.005], color:[0.85, 0.25, 0.25] }

block_1_5 (floor) 	{  Q:[-1.8, -0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_2_7 (floor) 	{  Q:[-1.4, 0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_2_10 (floor) 	{  Q:[-1.4, 1.8, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_3_1 (floor) 	{  Q:[-1, -1.8, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_3_6 (floor) 	{  Q:[-1, 0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_4_3 (floor) 	{  Q:[-0.6, -1, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_4_5 (floor) 	{  Q:[-0.6, -0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_5_7 (floor) 	{  Q:[-0.2, 0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }
block_6_5 (floor) 	{  Q:[0.2, -0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }

egoJoint (world) 	{  Q:[-0.5, -0.5, 0.1, 1, 0, 0, 0] }
ego (egoJoint) 	{  Q:[0.3, 0.3, 0, -1, 0, 0, 0], joint:transXY, limits:[-10, 10, -10, 10], shape:ssCylinder, size:[0.17, 0.17, 0.02], color:[0.96875, 0.742188, 0.308594], contact:1, logical:{ gripper }, sampleUniform:1 }

objJoint (world) 	{  Q:[0, 0, 0.1, 1, 0, 0, 0] }
obj (objJoint) 	{  Q:[0.2, 0.6, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.2, 0.2, 0.2, 0.02], color:[0, 0, 1], contact:1, mass:100, inertia:[0 0 0], logical:{ object } }

obj1Joint (world) 	{  Q:[0, 0, 0.1, 1, 0, 0, 0] }
obj1 (obj1Joint) 	{  Q:[0.6, 0.2, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.2, 0.2, 0.2, 0.02], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0], logical:{ object } }

block_9_3 (floor) 	{  Q:[1.4, -1, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[0.6953, 0.515625, 0.453125], contact:1, mass:100, inertia:[0 0 0] }

