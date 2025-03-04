Include: <../base-walls-min.g>

wall2_h (world){ shape:ssBox, Q:"t(1.6 -.1 0.3)", size:[.8 .1 0.6 .02], color:[0.6953 0.515625 .453125],contact: 1 }

egoJoint(world){Q:[0 0 0.1]  }
ego(egoJoint) {
    shape:ssCylinder, Q:[1.6 -1.3 0], size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{gripper}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}

goal (floor){ shape:ssBox, Q:"t(1.6 1.5 .0)", size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table} }
goal_visible (floor) 	{ shape:ssBox, Q:"t(1.6 1.5 0.1)", size:[0.2 0.2 0.1 0.005], color:[1. .3 .3] }

objJoint(world){ Q:[0.0 0.0 0.1] } # works
obj(objJoint) {
    shape:ssBox, Q:[-1.5 1.5 .0], size:[.3 .3 0.2 .02], logical={ object }  nomass:1, color:[0 0 1.0],
    joint:rigid, friction:.1  contact: 1
}


obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[0.5 -1 .0], size:[.2 1.5 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}
obj1_s1(obj1) {
    shape:ssBox, Q:[.6 0.7 0], size:[1.4 0.2 .2 .02], logical={ object } nomass:1,  color:[1 1 1], friction:.1  contact: 1
}
obj1_s2(obj1) {
    shape:ssBox, Q:[.6 0.5 0], size:[.2 .35 .2 .02], logical={ object } nomass:1,  color:[1 1 1], friction:.1  contact: 1
}


obj2Joint(world){ Q:[0.0 0.0 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, Q:[.8 -.65 .0], size:[.2 .4 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

obj3Joint(world){ Q:[0.0 0.0 0.1] } # works
obj3(obj3Joint) {
    shape:ssBox, Q:[.9 -1.3 .0], size:[.5 .55 .2 .02], logical={ object } nomass:1,  color:[1 1 1],
    joint:rigid, friction:.1  contact: 1
}

