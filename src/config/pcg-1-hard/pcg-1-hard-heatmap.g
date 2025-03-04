world 	{  X:[0, 0, 0.1, 1, 0, 0, 0] }
floor (world) 	{  Q:[0, 0, -0.05, 1, 0, 0, 0], shape:ssBox, size:[4.1, 4.1, 0.1, 0.04], color:[0 1 0], contact:1, friction:10, logical:{ table } }
outwall_right (world) 	{  Q:[0, -2, 0.2, 1, 0, 0, 0], shape:ssBox, size:[4.1, 0.1, 0.4, 0.04], color:[1, 1, 1], contact:1 }
outwall_back (world) 	{  Q:[2, 0, 0.2, 1, 0, 0, 0], shape:ssBox, size:[0.1, 4.1, 0.4, 0.04], color:[1, 1, 1], contact:1 }
outwall_left (world) 	{  Q:[0, 2, 0.2, 1, 0, 0, 0], shape:ssBox, size:[4.1, 0.1, 0.4, 0.04], color:[1, 1, 1], contact:1 }
outwall_front (world) 	{  Q:[-2, 0, 0.2, 1, 0, 0, 0], shape:ssBox, size:[0.1, 4.1, 0.4, 0.04], color:[1, 1, 1], contact:1 }
goal (floor) 	{  Q:[0.2, 1.8, 0, -1, 0, 0, 0], shape:ssBox, size:[0.35, 0.35, 0.1, 0.005], color:[1 1 1], logical:{ table } }
goal_visible (floor) 	{  Q:[0.2, 1.8, 0.1, -1, 0, 0, 0], shape:ssBox, size:[0.35, 0.35, 0.1, 0.005], color:[1 1 1] }
goalLarge (world) 	{  Q:[0.2, 1.8, -0.01, -1, 0, 0, 0], shape:ssBox, size:[0.2, 0.2, 0.025, 0.01], color:[1 1 1] }
block_2_6 (floor) 	{  Q:[-1.4, 0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_2_7 (floor) 	{  Q:[-1.4, 0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_3_2 (floor) 	{  Q:[-1, -1.4, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_3_4 (floor) 	{  Q:[-1, -0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_4_5 (floor) 	{  Q:[-0.6, -0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_5_4 (floor) 	{  Q:[-0.2, -1.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 1.6, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_5_7 (floor) 	{  Q:[-0.2, 0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_6_7 (floor) 	{  Q:[0.2, 0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_7_5 (floor) 	{  Q:[0.6, -0.2, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_8_7 (floor) 	{  Q:[1, 0.6, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }
block_9_8 (floor) 	{  Q:[1.4, 1, 0.2, -1, 0, 0, 0], shape:ssBox, size:[0.4, 0.4, 0.4, 0.01], color:[1, 1, 1], contact:1, mass:100, inertia:[0 0 0] }

#---------------------------------------------------------------------------------------------------#

egoJoint (world) 	{  Q:[-0.5, -0.5, 0.1, 1, 0, 0, 0] }
#---------------------------------------------------------------------------------------------------#

objJoint (world) 	{  Q:[0, 0, 0.1, -1, 0, 0, 0] }
obj (objJoint) 	{  Q:[-0.65, -1, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.2, 0.2, 0.2, 0.02], color:[1 1 1], contact:1, mass:100, inertia:[0 0 0], logical:{ object } }

#---------------------------------------------------------------------------------------------------#

obj1Joint (world) 	{  Q:[0, 0, 0.1, -1, 0, 0, 0] }
obj1 (obj1Joint) 	{  Q:[-1, -1, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.2, 0.2, 0.2, 0.02], color:[1, 0, 0], contact:1, mass:100, inertia:[0 0 0], logical:{ object } }

obj1_hm_r(obj1) {
    shape:ssBox, Q:[0 0 0], size:[0.2, 0.2, 0.6, 0.02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_b(world) {
    shape:ssBox, Q:[-1, -1 0.1], size:[0.2, 0.2, 0.3, 0.02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj1_hm_y(obj1) {
#    shape:ssBox, Q:[0 0 0], size:[0.3, 0.3, 0.4, 0.02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj1_cam_g(world): { rel: [-1, -1, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj1_cam_rel(obj1): { X: [-1, -1, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },

#---------------------------------------------------------------------------------------------------#

obj2Joint (world) 	{  Q:[0, 0, 0.1, -1, 0, 0, 0] }
obj2 (obj2Joint) 	{  Q:[-0.6, 0.2, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.2, 0.2, 0.2, 0.02], color:[0, 1, 0], contact:1, mass:100, inertia:[0 0 0], logical:{ object } }

obj2_hm_r(obj2) {
    shape:ssBox, Q:[0 0 0], size:[0.2, 0.2, 0.6, 0.02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_b(world) {
    shape:ssBox, Q:[-0.6, 0.2 0.1], size:[0.2, 0.2, 0.3, 0.02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj2_hm_y(obj2) {
#    shape:ssBox, Q:[0 0 0], size:[0.3, 0.3, 0.4, 0.02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj2_cam_g(world): { rel: [-0.6, 0.2, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj2_cam_rel(obj2): { X: [-0.6, 0.2, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },

#---------------------------------------------------------------------------------------------------#

obj3Joint (world) 	{  Q:[0, 0, 0.1, -1, 0, 0, 0] }
obj3 (obj3Joint) 	{  Q:[-1, -1.8, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.2, 0.2, 0.2, 0.02], color:[0, 0, 1], contact:1, mass:100, inertia:[0 0 0], logical:{ object } }

obj3_hm_r(obj3) {
    shape:ssBox, Q:[0 0 0], size:[0.2, 0.2, 0.6, 0.02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_b(world) {
    shape:ssBox, Q:[-1, -1.8 0.1], size:[0.2, 0.2, 0.3, 0.02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj3_hm_y(obj3) {
#    shape:ssBox, Q:[0 0 0], size:[0.3, 0.3, 0.4, 0.02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj3_cam_g(world): { rel: [-1, -1.8, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj3_cam_rel(obj3): { X: [-1, -1.8, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
