Include: <../base-walls-min-heatmap.g>
#---------------------------------------------------------------------------------------------------#

wall2_h (world){ shape:ssBox, Q:"t(-0.05 -1.2 0.3)", size:[2.5 .1 0.6 .02], color:[1 1 1],contact: 1 }
wall8_h (world){ shape:ssBox, Q:"t(-1.30 1.0 0.3)", size:[1.0 .1 0.6 .02], color:[1 1 1], contact: 1 }

#---------------------------------------------------------------------------------------------------#

egoJoint(world){ Q:[0.0 0.0 0.1] } # works

#---------------------------------------------------------------------------------------------------#

objJoint(world){ Q:[-1.2 1.3 0.1] } # works
obj(objJoint) { 
    type:ssBox size:[.3 .3 .2 .02] color:[1. 1. 1.],  logical={ object },
    joint:rigid, contact: 1 
}

#---------------------------------------------------------------------------------------------------#

obj1Joint(world){ Q:[1.4 -1 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, size:[.7 .1 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_r(obj1) {
    shape:ssBox, Q:[0 0 0], size:[.7 .1 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_b(world) {
    shape:ssBox, Q:[1.4 -1 0.1], size:[.7 .1 .3 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj1_hm_y(obj1) {
#    shape:ssBox, Q:[0 0 0], size:[.6 .2 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj1_cam_g(world): { rel: [1.4, -1, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj1_cam_rel(obj1): { X: [1.4, -1, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },

#---------------------------------------------------------------------------------------------------#

obj2Joint(world){ Q:[1.5 -0.85 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, size:[.7 .1 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_r(obj2) {
    shape:ssBox, Q:[0 0 0], size:[.7 .1 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_b(world) {
    shape:ssBox, Q:[1.5 -0.85 0.1], size:[.7 .1 .3 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj2_hm_y(obj2) {
#    shape:ssBox, Q:[0 0 0], size:[.8 .2 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj2_cam_g(world): { rel: [1.5, -0.85, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj2_cam_rel(obj2): { X: [1.5, -0.85, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },

#---------------------------------------------------------------------------------------------------#

obj3Joint(world){ Q:[1.1 -1.6 0.1] } # works
obj3(obj3Joint) {
    shape:ssBox, size:[.1 .5 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_r(obj3) {
    shape:ssBox, Q:[0 0 0], size:[.15 .6 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_b(world) {
    shape:ssBox, Q:[1.1 -1.6 0.1], size:[.1 .5 .3 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}
#obj3_hm_y(obj3) {
#    shape:ssBox, Q:[0 0 0], size:[.2 .6 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
#    joint:rigid, friction:.1  contact: 1
#}
obj3_cam_g(world): { rel: [1.1, -1.6, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj3_cam_rel(obj3): { X: [1.1, -1.6, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },

#---------------------------------------------------------------------------------------------------#


obj4Joint(world){ Q:[0 0 0.1] } # works
obj4(obj4Joint) {
    shape:ssBox, Q:[0 -1.6 0], size:[.2 .2 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj4_hm_r(obj4) {
    shape:ssBox, Q:[0 0 0], size:[.25 .3 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj4_hm_b(world) {
    shape:ssBox, Q:[0 -1.6 0.1], size:[.2 .2 .3 .02], logical={ object } nomass:1,  color:[0 0 1 ],
    joint:rigid, friction:.1  contact: 1
}

obj4_cam_g(world): { rel: [0 -1.6, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
obj4_cam_rel(obj4): { X: [0 -1.6, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 30, height: 30, focalLength: 1, zRange: [0.5, 3]  },
