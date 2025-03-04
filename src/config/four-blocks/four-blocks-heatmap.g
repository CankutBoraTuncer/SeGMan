Include: <../base-walls-min-heatmap.g>
wall2_h (world){ shape:ssBox, Q:"t(1 -.75 0.3)", size:[.1 2.5 0.6 .02], color:[1 0 0],contact: 1 }

egoJoint(world){ Q:[0.0 0.0 0.1] } # works

obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[-1.2 -0.3 .0], size:[1 0.2 .01 .02], logical={ object } nomass:1,  color:[0 0 1.0],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_r(obj1) {
    shape:ssBox, Q:[0 0 0], size:[1 0.2 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_b(world) {
    shape:ssBox, Q:[-1.2 -0.3 0.1], size:[1 0.2 .2 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj1_cam_g(world): { rel: [-1.2, -0.3, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj1_cam_rel(obj1): { X: [-1.2, -0.3, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },


obj2Joint(world){ Q:[0.0 0.0 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, Q:[-0.75 -1.2 .0], size:[.2 1 .01 .02], logical={ object } nomass:1,  color:[0 1.0 0],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_r(obj2) {
    shape:ssBox, Q:[0 0 0], size:[.2 1 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_b(world) {
    shape:ssBox, Q:[-0.75 -1.2 .1], size:[.2 1 .1 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj2_cam_g(world): { rel: [-0.75, -1.2, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj2_cam_rel(obj2): { X: [-0.75, -1.2, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },

obj3Joint(world){ Q:[0.0 0.0 0.1] } # works
obj3(obj3Joint) {
    shape:ssBox, Q:[-0.5 -1.4 .0], size:[.2 1 .01 .02], logical={ object } nomass:1,  color:[1 1.0 1.0],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_r(obj3) {
    shape:ssBox, Q:[0 0 0], size:[.2 1 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_b(world) {
    shape:ssBox, Q:[-0.5 -1.4 0.1], size:[.2 1 .1 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj3_cam_g(world): { rel: [-0.5, -1.4, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj3_cam_rel(obj3): { X: [-0.5, -1.4, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },


obj4Joint(world){ Q:[0.0 0.0 0.1] } # works
obj4(obj4Joint) {
    shape:ssBox, Q:[-1.4 -0.75 .0], size:[1 0.2 .01 .02], logical={ object } nomass:1,  color:[1.0 0 0],
    joint:rigid, friction:.1  contact: 1
}
obj4_hm_r(obj4) {
    shape:ssBox, Q:[0 0 0], size:[1 0.2 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj4_hm_b(world) {
    shape:ssBox, Q:[-1.4 -0.75 .1], size:[1 0.2 .1 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj4_cam_g(world): { rel: [-1.4, -0.75, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj4_cam_rel(obj4): { X: [-1.4, -0.75, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },

objJoint(world){ Q:[0.0 0.0 0.1] } # works



