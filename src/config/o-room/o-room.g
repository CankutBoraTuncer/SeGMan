Include: <../base-walls-min.g>

egoJoint(world){ Q:[0 0.0 0.1] } # works
ego(egoJoint) {
    shape:ssCylinder, Q:[1.7 0.0 0], size:[.2 .2 .02], color:[0.96875 0.7421875 0.30859375], logical:{gripper}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}


goal (floor){ shape:ssBox, Q:"t(-1.6 1.6 .0)", size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table} }

goal_visible (floor) 	{ shape:ssBox, Q:"t(-1.6 1.6 0.1)", size:[0.2 0.2 0.1 0.005], color:[1. .3 .3] }


wall1_v (world){ shape:ssBox, Q:"t(1.4 0.0 0.3)", size:[0.1 2.6 0.6 .02], color:[0.6953 0.515625 .453125],  contact: 1 }
wall2_h (world){ shape:ssBox, Q:"t(0.0 -1.25 0.3)", size:[2.65 .1 0.6 .02], color:[0.6953 0.515625 .453125],  contact: 1 }
wall4_h (world){ shape:ssBox, Q:"t(0.0 1.25 0.3)", size:[2.65 .1 0.6 .02], color:[0.6953 0.515625 .453125],  contact: 1 }

wall5_v (world){ shape:ssBox, Q:"t(-1.4 0.0 0.3)", size:[0.1 2.6 0.6 .02], color:[0.6953 0.515625 .453125],  contact: 1 }

objJoint(world){ Q:[1.5 -1.5 0.1] } # works
obj(objJoint) {type:ssBox size:[.3 .3 .2 .02] color:[0. 0. 1.],  logical={ object }, joint:rigid, contact: 1 }

goalLarge (world){ shape:ssBox, Q:"t(-1.6 1.6 -0.01)", size:[0.2 0.2 0.025, 0.01], color:[1. .3 .3]}

