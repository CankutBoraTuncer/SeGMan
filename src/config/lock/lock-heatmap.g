Include: <../base-walls-min-heatmap.g>
wall2_h (world){ shape:ssBox, Q:"t(1.6 -.1 0.3)", size:[.8 .1 0.6 .02], color:[1 0 0 ],contact: 1 }

egoJoint(world){Q:[0 0 0.1]  }
objJoint(world){ Q:[0.0 0.0 0.1] } # works

#-------------------------------------------------------------------------------------------------------------------------------#
obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[0.5 -1 .0], size:[.2 1.5 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_r(obj1) {
    shape:ssBox, Q:[0 0 0], size:[.2 1.5 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_b(world) {
    shape:ssBox, Q:[0.5 -1 0.1], size:[.2 1.5 .4 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

ob1_s1(obj1) {
    shape:ssBox, Q:[.6 0.7 0], size:[1.4 0.2 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
ob1_s1_hm_r(ob1_s1) {
    shape:ssBox, Q:[0 0 0], size:[1.4 0.2 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
ob1_s1_hm_b(obj1) {
    shape:ssBox, Q:[.6 0.7 0], size:[1.4 0.2 .4 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

ob1_s2(obj1) {
    shape:ssBox, Q:[.6 0.5 0], size:[.2 .35 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
ob1_s2_hm_r(ob1_s2) {
    shape:ssBox, Q:[0 0 0], size:[.2 .35 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
ob1_s2_hm_b(obj1) {
    shape:ssBox, Q:[.6 0.5 0], size:[.2 .35 .4 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj1_cam_g(world): { rel: [.8, -.7, 3.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj1_cam_rel(obj1): { X: [.8, -.7, 3.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },

#-------------------------------------------------------------------------------------------------------------------------------#
obj2Joint(world){ Q:[0.0 0.0 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, Q:[.8 -.65 .0], size:[.2 .4 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj2_hm_r(obj2) {
    shape:ssBox, Q:[0 0 0], size:[.2 .4 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_b(world) {
    shape:ssBox, Q:[.8 -.65 0.1], size:[.2 .4 .4 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj2_cam_g(world): { rel: [.8 -.65, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj2_cam_rel(obj2): { X: [.8 -.65, 3.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
#-------------------------------------------------------------------------------------------------------------------------------#

obj3Joint(world){ Q:[0.0 0.0 0.1] } # works
obj3(obj3Joint) {
    shape:ssBox, Q:[.9 -1.3 .0], size:[.5 .55 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj3_hm_r(obj3) {
    shape:ssBox, Q:[0 0 0], size:[.5 .55 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_b(world) {
    shape:ssBox, Q:[.9 -1.3 0.1], size:[.5 .55 .4 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj3_cam_g(world): { rel: [.9 -1.3, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },
obj3_cam_rel(obj3): { X: [.9 -1.3, 3.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: camera, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.2, 5]  },

#-------------------------------------------------------------------------------------------------------------------------------#



