#This code is used to transform a csv file describing an orbit into a set of images. 
#It is the THALASSA module concerning with generating synthetic sensor acquisitions. 

#Source data is the path to a .txt document structured as a csv. 
#The .txt document contains information about the orbit (camera position/attitude), and the Sun.
#In particular, in this iteration the .txt is structured as follows: 
# X | Y | Z | αX | αY | αZ | φ
#The Sun is considered moving in the Equator in a circular motion. Its angular position is φ. 
#X, Y and Z define the camera position in Blender. 
#αX, αY and αZ define the camera attitude in Blender. 
#Phi (φ) and all the other angles should be provided in radians 

#Other modules of THALASSA provide the capability to generate this file automatically from initial orbital conditions.  
 
#The target folder is the folder where the images are saved

# Some sources used in the construction of this code: 
# https://blender.stackexchange.com/questions/151319/adding-camera-to-scene
# https://stackoverflow.com/questions/17355617/can-you-add-a-light-source-in-blender-using-python

import os, math, csv, bpy
import numpy as np 
#IF NOTHING IS SHOWING IN IMAGES IT'S HIGHLY LIKELY A CLIPPING ISSUE. INCREASE CLIPPING DISTANCE IF THAT'S THE CASE

SourceData = 'your/source/folder/here.txt'
TargetFolder = 'your/target/folder/here'


for C in bpy.context.scene.objects:
    if C.type == 'CAMERA':
        C.select_set(True)
    else:
        C.select_set(False)
bpy.ops.object.delete()        
        
for L in bpy.context.scene.objects:
    if L.type == 'LIGHT':
        L.select_set(True)
    else:
        L.select_set(False)
        
bpy.ops.object.delete()

bpy.context.scene.render.engine = 'CYCLES' #Render engine - Eevee was producing a strange light bleed
#bpy.context.scene.cycles.device = 'GPU'

RL = 12000 #Distance at which the Sun is considered 
FocalLength = 9.6 #Focal length of the camera, in mm
bpy.data.worlds["World"].node_tree.nodes["Background"].inputs[0].default_value = (0, 0, 0, 1)


#Create camera and Sun
CamSat = bpy.data.cameras.new("Satellite Camera")
CamSat.lens = FocalLength
CamSat.clip_end = 40000 #REMEMBER TO CHANGE CLIPPING IF NOTHING APPEARS!
dLight = bpy.data.lights.new(name="SunLight", type='SUN')
dLight.energy = 30

#Create camera and Sun objects
Sunlight = bpy.data.objects.new(name="Sun", object_data=dLight)
CamSat = bpy.data.objects.new("Satellite Camera", CamSat)

bpy.context.collection.objects.link(Sunlight)
bpy.context.collection.objects.link(CamSat)

bpy.context.view_layer.objects.active = Sunlight
bpy.context.view_layer.objects.active = CamSat

dg = bpy.context.evaluated_depsgraph_get() 
dg.update()

#Add your folder here 
with open(SourceData, newline='') as txtinput: #Write source folder here 
    States_expanded_string = list(csv.reader(txtinput))
    
States_expanded =  np.array(States_expanded_string, dtype=np.float32)

for timestep in range(0,len(States_expanded)):
    CamSat.location.x = States_expanded[timestep][0]
    CamSat.location.y = States_expanded[timestep][1]
    CamSat.location.z = States_expanded[timestep][2]
    CamSat.rotation_euler[0] = States_expanded[timestep][3]
    CamSat.rotation_euler[1] = States_expanded[timestep][4]
    CamSat.rotation_euler[2] = States_expanded[timestep][5]
    Phi = States_expanded[timestep][6]
    Sunlight.location[0] = RL*math.cos(Phi) #Change here to have a non-equatorial Sun
    Sunlight.location[1] = RL*math.sin(Phi) #Change here to have a non-equatorial Sun
    Sunlight.location[2] = 0                #Change here to have a non-equatorial Sun
    Sunlight.rotation_euler[0] = 0
    Sunlight.rotation_euler[1] = 0.5*math.pi
    Sunlight.rotation_euler[2] = Phi
    #Removed pi from rot.Z Sun
    scene = bpy.context.scene
    
    for CAMM in scene.objects:
        if CAMM.type == 'CAMERA':
            bpy.context.scene.camera = CAMM
            bpy.context.scene.render.resolution_x = 1280
            bpy.context.scene.render.resolution_y = 720

            file = TargetFolder + str(timestep+1) #Write target folder here 
            bpy.context.scene.render.filepath = file
            bpy.ops.render.render( write_still=True) 



