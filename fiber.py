# coding=UTF-8
from abaqusConstants import *
from caeModules import *
import numpy as np
import math

def intersection (a,b):
	a1 = list(a[0])
	a2 = list(a[1])
	b1 = list(b[0])
	b2 = list(b[1])

	v1 = np.cross(np.array(a1) - np.array(b1), np.array(b2) - np.array(b1))
	v2 = np.cross(np.array(a2) - np.array(b1), np.array(b2) - np.array(b1))
	dot_value = np.dot(v1, v2)
	v11 = np.cross(np.array(b1) - np.array(a1), np.array(a2) - np.array(a1))
	v21 = np.cross(np.array(b2) - np.array(a1), np.array(a2) - np.array(a1))
	dot_value1 = np.dot(v11, v21)

	if dot_value < 0 and dot_value1 < 0:
		return 1
	else:
		return 0

def fiberlocation (b):
	Psi = np.random.uniform(0,2 * math.pi)
	Theta = np.random.uniform(0,2 * math.pi)
	x = b[0] + fiberlength * math.sin(Psi) * math.sin(Theta)
	y = b[1] + fiberlength * math.sin(Psi) * math.cos(Theta)
	z = b[2] + fiberlength * math.cos(Psi)
	endpoint = (x,y,z)
	return endpoint


L = 150.0
H = 150.0
B = 550.0
fiberlength = 12
fibernumber = 800

fiber = [] * fibernumber
initialfiber = (np.random.uniform(-(H/2-fiberlength),(H/2-fiberlength)),np.random.uniform(-(B/2-fiberlength),(B/2-fiberlength)),np.random.uniform(-(L/2-fiberlength),(L/2-fiberlength)))
fiber.append((initialfiber,fiberlocation(initialfiber)))

i = 1
while i < fibernumber:
	ipoint = (np.random.uniform(-H/2,H/2),np.random.uniform(-B/2,B/2),np.random.uniform(-L/2,L/2))
	epoint = fiberlocation(ipoint)
	if abs(epoint[0])<H/2 and abs(epoint[1])<B/2 and abs(epoint[2])<L/2:
		for j in range(0, i):
			signal = intersection(fiber[j], (ipoint, epoint))
			if signal is True:
				break
		fiber.append((ipoint, epoint))
		i = i + 1
	else:
		continue

myModel = mdb.Model(name='Model-1')
p = myModel.Part(name='fiber', dimensionality=THREE_D, type=DEFORMABLE_BODY)
p.ReferencePoint(point=(0.0, 0.0, 0.0))

for j in range (0,fibernumber):
	p.WirePolyLine(points=(fiber[j][0], fiber[j][1]), mergeType=IMPRINT, meshable=ON)

del p.features['RP']

#构建混凝土部分
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.Line(point1=(-H/2, -B/2), point2=(-H/2, B/2))
s.VerticalConstraint(entity=g[2], addUndoState=False)
s.Line(point1=(-H/2, B/2), point2=(H/2, B/2))
s.HorizontalConstraint(entity=g[3], addUndoState=False)
s.PerpendicularConstraint(entity1=g[2], entity2=g[3], addUndoState=False)
s.Line(point1=(H/2, B/2), point2=(H/2, -B/2))
s.VerticalConstraint(entity=g[4], addUndoState=False)
s.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
s.Line(point1=(H/2, -B/2), point2=(-H/2, -B/2))
s.HorizontalConstraint(entity=g[5], addUndoState=False)
s.PerpendicularConstraint(entity1=g[4], entity2=g[5], addUndoState=False)
p = mdb.models['Model-1'].Part(name='Concrete', dimensionality=THREE_D, type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts['Concrete']
p.BaseSolidExtrude(sketch=s, depth=L)
s.unsetPrimaryObject()
session.viewports['Viewport: 1'].setValues(displayedObject=p)
del mdb.models['Model-1'].sketches['__profile__']

#组合拼装
a = mdb.models['Model-1'].rootAssembly
a.DatumCsysByDefault(CARTESIAN)
p = mdb.models['Model-1'].parts['Concrete']
a.Instance(name='Concrete-1', part=p, dependent=ON)
a.translate(instanceList=('Concrete-1', ), vector=(0.0, 0.0, -L/2))
p = mdb.models['Model-1'].parts['fiber']
a.Instance(name='fiber-1', part=p, dependent=ON)
