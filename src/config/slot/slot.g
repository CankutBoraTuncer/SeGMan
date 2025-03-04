Include: <../base-walls-min.g>

goal (floor){ shape:ssBox, Q:"t(1.6 -.4 .0)", size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table} }
goal_visible (floor) 	{ shape:ssBox, Q:"t(1.6 -.4 0.1)", size:[0.2 0.2 0.1 0.005], color:[1. .3 .3] }
wall_h (world){ shape:ssBox, Q:"t(1.09 -0.9 0.3)", size:[1.82 .1 0.6 .02], color:[0.6953 0.515625 .453125],contact: 1 }
wall1_v (world){ shape:ssBox, Q:"t(0.2 1.5 0.3)", size:[0.1 1 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }
wall2_v (world){ shape:ssBox, Q:"t(0.2 -0.4 0.3)", size:[0.1 1 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }


egoJoint(world){ Q:[0 0.0 0.1] } # works
ego(egoJoint) {
    shape:ssCylinder, Q:[1.6 -1.6 0], size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{gripper}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}

obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[0.0 0.6 .0], size:[.2 .5 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj2Joint(world){ Q:[0.0 0.0 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, Q:[-1 1.4 .0], size:[.2 .6 .2 .02], logical={ object } nomass:1,  color:[1 1.0 1],
    joint:rigid, friction:.1  contact: 1
}

obj3Joint(world){ Q:[0.0 0.0 0.1] } # works
obj3(obj3Joint) {
    shape:ssBox, Q:[-0.4 1.4 .0], size:[.2 1 .2 .02], logical={ object } nomass:1,  color:[1.0 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj4Joint(world){ Q:[0.0 0.0 0.1] } # works
obj4(obj4Joint) {
    shape:ssBox, Q:[-1.5 -1.4 .0], size:[.2 0.5 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj5Joint(world){ Q:[0.0 0.0 0.1] } # works
obj5(obj1Joint) {
    shape:ssBox, Q:[0.4 -1.45 .0], size:[.2 .8 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

objJoint(world){ Q:[0 0 0.1] } # works
obj(objJoint) { type:ssBox size:[.3 .3 .2 .02] Q:"t(-1.6 1.6  0)" color:[0 0 1],  logical={ object }, joint:rigid, contact: 1 }

