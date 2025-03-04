Include: <../base-walls-min-heatmap.g>

goal (floor){ shape:ssBox, Q:"t(-1.3 -1.3 .0)", size:[0.2 0.2 .1 .005], color:[1 1 1], contact:0, logical:{table} }
goal_visible (floor) 	{ shape:ssBox, Q:"t(-1.3 -1.3 0.1)", size:[0.2 0.2 0.1 0.005], color:[1 1 1] }

egoJoint(world){ Q:[0.0 0.0 0.1] } # works

obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[-0.75 -1.3 .0], size:[.2 0.9 .2 .02], logical={ object } nomass:1,  color:[0 1.0 0],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_r(obj1) {
    shape:ssBox, Q:[0 0 0], size:[.2 0.9 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_b(world) {
    shape:ssBox, Q:[-0.75 -1.3 .1], size:[.2 0.9 .1 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj1_hm_y(obj1) {
#    shape:ssBox, Q:[0 0 0], size:[.35 1.15 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj1_cam_g(world): { rel: [-0.75, -1.3, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj1_cam_rel(obj1): { X: [-0.75, -1.3, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },

obj2Joint(world){ Q:[0.0 0.0 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, Q:[-1.4 -0.75 .0], size:[.9 0.2 .2 .02], logical={ object } nomass:1,  color:[1.0 0 0],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_r(obj2) {
    shape:ssBox, Q:[0 0 0], size:[.9 0.2 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_b(world) {
    shape:ssBox, Q:[-1.4 -0.75 .1], size:[.9 0.2 .1 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj2_hm_y(obj2) {
#    shape:ssBox, Q:[0 0 0], size:[1.15 0.35 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj2_cam_g(world): { rel: [-1.4, -0.75, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj2_cam_rel(obj2): { X: [-1.4, -0.75, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },




objJoint(world){ Q:[0.0 0.0 0.1] } # works
obj(objJoint) {
    shape:ssBox, Q:[1.6 -0.5 .0], size:[.3 .3 0.2 .02], logical={ object }  nomass:1, color:[0 0 1.0],
    joint:rigid, friction:.1  contact: 1
}


